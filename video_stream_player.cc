#include <sys/types.h>
#include <stdio.h>
#include <iostream>
#include <functional>

#ifdef _WIN32
#include <winsock2.h>
typedef int socklen_t;
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
typedef int SOCKET;
typedef struct sockaddr SOCKADDR;
#define closesocket(s) close(s)
#endif

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include <SDL.h>
#include <SDL_thread.h>

#include <boost/program_options.hpp>


class FFmpegStreamDestination {
public:
  FFmpegStreamDestination(AVCodecID decoder) : m_sws_ctx(0) {

    avcodec_register_all();

    // Allocate the packet
    if (!(m_packet = av_packet_alloc())) {
      std::cerr << "Error allocating a packet" << std::endl;
      exit(1);
    }

    // Find the decoder
    if (!(m_pCodec = avcodec_find_decoder(decoder))) {
      std::cerr << "Error finding the decoder" << std::endl;
      exit(1);
    }

    // Create the parser
    if (!(m_parser = av_parser_init(m_pCodec->id))) {
      std::cerr << "Error creating the parser" << std::endl;
      exit(1);
    }

    // Allocate the context
    if (!(m_pCodecCtx = avcodec_alloc_context3(m_pCodec))) {
      std::cerr << "Error opening the decoding context" << std::endl;
      exit(1);
    }

    // Open the codec
    if (avcodec_open2(m_pCodecCtx, m_pCodec, 0) < 0) {
      std::cerr << "Error opening the decoding codec" << std::endl;
      exit(1);
    }

    // Allocate the YUV frame.
    m_pFrame = av_frame_alloc();
  }

  ~FFmpegStreamDestination() {
    av_parser_close(m_parser);
    avcodec_close(m_pCodecCtx);
    av_free(m_pFrame);
    av_packet_unref(m_packet);
    SDL_DestroyTexture(m_texture);
    SDL_DestroyRenderer(m_renderer);
    SDL_DestroyWindow(m_screen);
  }

  bool decode(unsigned char *pData, size_t sz) {
    uint8_t *data = pData;
    size_t data_size = sz;
    static bool first = false;
    while (data_size > 0) {
      if (first) {
	first = false;
	return true;
      }

      // Trye to parse a frame out of the buffer
      //printf("%d %d %d %d %d\n", data[0], data[1], data[2], data[3], data_size); fflush(stdout);

      int consumed = av_parser_parse2(m_parser, m_pCodecCtx, &(m_packet->data), &(m_packet->size),
				      data, data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);

      // Move the data pointer passed the consumed bytes
      data += consumed;
      data_size -= consumed;

      // Decode the packet.
      if (m_packet->size) {
	if (avcodec_send_packet(m_pCodecCtx, m_packet) < 0) {
	  std::cerr << "Error in avcodec_send_packet" << std::endl;
	  return false;
	}
	int ret = avcodec_receive_frame(m_pCodecCtx, m_pFrame);
	if (ret < 0) {
	  return ((ret != AVERROR(EAGAIN)) && (ret != AVERROR_EOF));
	}

	// Did we decoded a frame parsed out of the stream
	int width = m_pCodecCtx->width;
	int height = m_pCodecCtx->height;

	// Allocate the conversion context
	if (m_sws_ctx == 0) {

	  // initialize SWS context for software scaling
	  m_sws_ctx = sws_getContext(width, height, m_pCodecCtx->pix_fmt, width, height,
				     AV_PIX_FMT_YUV420P, SWS_BILINEAR, NULL, NULL, NULL);

	  // Create the playback window.
	  m_screen = SDL_CreateWindow("UDP Video Player", SDL_WINDOWPOS_UNDEFINED,
				      SDL_WINDOWPOS_UNDEFINED, width, height, 0);

	  // Create the renderer
	  m_renderer = SDL_CreateRenderer(m_screen, -1, 0);
	  if (!m_renderer) {
	    std::cerr << "SDL: cound not create the SDL renderer\n";
	    return false;
	  }

	  // Allocate a place to put our YUV image on that screen
	  m_texture = SDL_CreateTexture(m_renderer, SDL_PIXELFORMAT_YV12, SDL_TEXTUREACCESS_STREAMING, width, height);
	  if (!m_texture) {
	    std::cerr << "SDL: could not create the SDL texture\n";
	    return false;
	  }

	  // set up YV12 pixel array (12 bits per pixel)
	  int y_plane_size = width * height;
	  int uv_plane_size = width * height / 4;
	  m_y_plane = (uint8_t*)malloc(y_plane_size);
	  m_u_plane = (uint8_t*)malloc(uv_plane_size);
	  m_v_plane = (uint8_t*)malloc(uv_plane_size);
	  if (!m_y_plane || !m_u_plane || !m_v_plane) {
	    std::cerr << "Could not allocate the pixel buffers\n";
	    return false;
	  }
	}

	// Convert the image into YUV format that SDL uses
	AVFrame pict;
	int uv_pitch = width / 2;
	pict.data[0] = m_y_plane;
	pict.data[1] = m_u_plane;
	pict.data[2] = m_v_plane;
	pict.linesize[0] = width;
	pict.linesize[1] = uv_pitch;
	pict.linesize[2] = uv_pitch;
	sws_scale(m_sws_ctx, (uint8_t const * const *)m_pFrame->data,
		  m_pFrame->linesize, 0, height, pict.data, pict.linesize);

	  // Update the texture
	  SDL_UpdateYUVTexture(m_texture, NULL, m_y_plane, width, m_u_plane, uv_pitch, m_v_plane, uv_pitch);

	  // Render the texture
	  SDL_RenderClear(m_renderer);
	  SDL_RenderCopy(m_renderer, m_texture, NULL, NULL);
	  SDL_RenderPresent(m_renderer);
	}
      }
  
    return true;
  }
  
protected:
  AVCodecContext  *m_pCodecCtx;
  AVCodec *m_pCodec;
  AVCodecParserContext *m_parser;
  AVPacket *m_packet;
  AVFrame *m_pFrame;
  struct SwsContext *m_sws_ctx;
  SDL_Window *m_screen;
  SDL_Renderer *m_renderer;
  SDL_Texture *m_texture;
  uint8_t *m_y_plane;
  uint8_t *m_u_plane;
  uint8_t *m_v_plane;
};

int main(int argc, char* argv[]) {
  namespace po=boost::program_options;
  std::string hostname;
  uint32_t port_num;
  uint32_t packet_size;
  bool use_udp;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("hostname,h", po::value<std::string>(&hostname)->default_value("vtx1"),
     "name or IP address of the video server")
    ("port,p", po::value<uint32_t>(&port_num)->default_value(2470),
     "port number of the video server (default 2470)")
    ("packet_size,s", po::value<uint32_t>(&packet_size)->default_value(32767),
     "the size of the packet buffer (the maximum size of a packet)")
    ("use_udp,U", po::bool_switch(&use_udp),
     "use the UDP protocol rather than TCP")
    ;

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << "Usage: options_description [options]\n";
    std::cout << desc;
    return 0;
  }

#ifdef _WIN32
  // Initialize the socket interface
  WSADATA wsaData;
  WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

  // Create a recieve socket
  struct sockaddr_in local;
  bzero((char *)&local, sizeof(local));
  local.sin_family = AF_INET;
  local.sin_port = htons(port_num);
  SOCKET sock = socket(AF_INET, use_udp ? SOCK_DGRAM : SOCK_STREAM, use_udp ? 0 : IPPROTO_TCP);
  if (sock < 0) {
    std::cerr << "Error opening the socket" << std::endl;
    exit(-1);
  }
  if (use_udp) {
    local.sin_addr.s_addr = INADDR_ANY;
    sock = 

    // Bind to the UDPsocket
    bind(sock, (sockaddr*)&local, sizeof(local));
  } else {
#ifdef _WIN32
    local.sin_addr.s_addr = inet_addr(IP_ADDRESS);

    // Connect to the TCP socket
    if (connect(sock, (SOCKADDR *)&local, sizeof(local)) == SOCKET_ERROR) {
      std::cerr << "Error connecting to the server "
		<< hostname << ":" << port_num << std::endl;
      exit(1);
    }
#else
    struct hostent *server = gethostbyname(hostname.c_str());
    if (server == NULL) {
      std::cerr << "ERROR, no such host: " << hostname << std::endl;
      exit(-1);
    }
    local.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&local.sin_addr.s_addr, server->h_length);

    // Connect to the TCP socket
    if (connect(sock, (SOCKADDR *)&local, sizeof(local)) < 0) {
      std::cerr << "Error connecting to the server "
		<< hostname << ":" << port_num << std::endl;
      exit(1);
    }
  }
#endif

  // Create the h264 decoder class.
  FFmpegStreamDestination dec(AV_CODEC_ID_H264);

  // Initialize SDL for video output.
  if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)) {
    std::cerr << "Could not initialize SDL - " << SDL_GetError() << std::endl;
    exit(1);
  }

  uint8_t *buffer = new uint8_t[packet_size];
  bool done = false;
  while (!done) {

    // Try to read UDP packets
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    int rec_len = recvfrom(sock, reinterpret_cast<char*>(buffer), packet_size, 0,
			   (sockaddr*)&from, &from_len);
    if (rec_len > 0) {
      //printf("%d %d %d %d %d\n", buffer[0], buffer[1], buffer[2], buffer[3], rec_len); fflush(stdout);
      dec.decode(buffer, rec_len);
    }

    // Check for SDL events.
    SDL_Event event;
    SDL_PollEvent(&event);
    switch (event.type) {
    case SDL_QUIT:
      SDL_Quit();
      done = true;
      break;
    default:
      break;
    }
  }

  // Free the packet buffer
  delete [] buffer;

  // Close the socket
  closesocket(sock);

  return 0;
}
