
#include <sys/types.h>
#include <stdio.h>
#include <iostream>
#include <functional>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fcntl.h>

#ifdef __WIN32
#include <winsock2.h>
#include <shlwapi.h>
#endif

#include <srt.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/asio.hpp>

#include "telemetry.hh"
#include "texture.hh"
#include "ffmpeg_decoder.hh"
#include "sdl_render_window.hh"
#include "transmitter.hh"
#include "fec.h"

namespace po=boost::program_options;
namespace ip=boost::asio::ip;

static bool g_quit = false;

bool check_for_quit() {
  // Check for the SDL quit event.
  SDL_Event event;
  SDL_PollEvent(&event);
  if (event.type == SDL_QUIT) {
    g_quit = true;
  }
  return g_quit;
}

int main(int argc, char* argv[]) {
  std::string hostname;
  uint16_t port;
  uint32_t packet_size;
  bool use_udp;
  bool use_srt;
  bool use_fec;
  uint16_t data_per_fec_block;
  uint16_t fec_per_fec_block;
  uint32_t win_x;
  uint32_t win_y;
  bool windowed;
  uint8_t screen;
  std::string url;
  std::string font_file;
  std::string icon_dir;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("hostname,h", po::value<std::string>(&hostname), "name or IP address of the video server")
    ("port,p", po::value<uint16_t>(&port)->default_value(0), "port number of the video server")
    ("packet_size", po::value<uint32_t>(&packet_size)->default_value(32767),
     "the size of the packet buffer (the maximum size of a packet)")
    ("fec,f", po::bool_switch(&use_fec),
     "the packets have been encoded with forward error correction")
    ("data_per_block,b", po::value<uint16_t>(&data_per_fec_block)->default_value(8),
     "number of data packets per FEC block")
    ("fec_per_block,,r", po::value<uint16_t>(&fec_per_fec_block)->default_value(4),
     "number of FEC packets per FEC block")
    ("use_udp,U", po::bool_switch(&use_udp), "use the UDP protocol rather than TCP")
    ("use_srt,S", po::bool_switch(&use_srt), "use the SRT protocol rather than TCP/UDP")
    ("win_x", po::value<uint32_t>(&win_x)->default_value(SDL_WINDOWPOS_UNDEFINED),
     "the X component of the starting location of the window")
    ("win_y", po::value<uint32_t>(&win_y)->default_value(SDL_WINDOWPOS_UNDEFINED),
     "the Y component of the starting location of the window")
    ("windowed,W", po::bool_switch(&windowed), "run the program in windowed mode")
    ("screen,s", po::value<uint8_t>(&screen)->default_value(0),
     "the screen to display the video on")
    ("url,u", po::value<std::string>(&url), "read from the specified URL")
    ("font", po::value<std::string>(&font_file), "the path to the OSD font file")
    ("icon_dir", po::value<std::string>(&icon_dir),
     "the path to the directory containing the OSD textures")
    ;

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << "Usage: options_description [options]\n";
    std::cout << desc;
    return 0;
  }
  bool use_url = (url.length() > 0);

  // Create the boost asio IO context for the network interfaces.
#if BOOST_VERSION < 106600
  boost::asio::io_service io_context;
#else
  boost::asio::io_context io_context;
#endif

#ifdef __WIN32
  // Get the application path
  char fname[1024];
  GetModuleFileNameA(NULL, fname, 1024);
  try {
    boost::filesystem::path exe_fname(fname);
    boost::filesystem::path bin_dir = exe_fname.parent_path();
    boost::filesystem::path inst_dir = bin_dir.parent_path();

    // Default the icon directory if it's not set
    if (icon_dir.empty()) {
      icon_dir = (inst_dir / "icons").string();
    }

    // Default the font file if it's not set
    if (font_file.empty()) {
      font_file = (inst_dir / "fonts/Cousine-Regular.ttf").string();
    }

  } catch (const std::exception &e) {
  }
#endif

  // Initialize SDL for video output and joystick.
  if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK)) {
    std::cerr << "Could not initialize SDL - " << SDL_GetError() << std::endl;
    SDL_Quit();
    exit(1);
  }

  // Create the interface to read from the transmitter;
  Transmitter tx;

  // Create the address resolver
  ip::tcp::resolver tcp_resolver(io_context);

  // Create the telemetry class
  std::shared_ptr<Telemetry> telem(new Telemetry(io_context, tx));

  // Create the class for rendering everything
  SDLRenderWindow win(telem, font_file, icon_dir, win_x, win_y, screen, windowed);

  // Create the draw callback
  std::mutex m;
  std::condition_variable cv;
  uint16_t size[2];
  uint8_t *planes[3];
  auto draw_cb = [&m, &cv, &planes, &size]
    (uint32_t width, uint32_t height, uint8_t *y_plane, uint8_t *u_plane, uint8_t *v_plane) {
		   size[0] = width;
		   size[1] = height;
		   planes[0] = y_plane;
		   planes[1] = u_plane;
		   planes[2] = v_plane;
		   cv.notify_all();
		 };
  std::thread draw_thread([&win, &m, &cv, &planes, &size]() {
			    while(1) {
			      std::unique_lock<std::mutex> lk(m);
			      cv.wait(lk);
			      win.update(size[0], size[1], planes[0], planes[1], planes[2]);
			      if (check_for_quit()) {
				return;
			      }
			    }
			  });

  std::shared_ptr<FFMpegDecoder> dec;
  if (use_url) {

    // Create the decoder class
    dec.reset(new FFMpegDecoder(url, draw_cb));

    while (dec->decode_url() && !check_for_quit()) {}

  } else {

    // Create the decoder class
    dec.reset(new FFMpegDecoder(AV_CODEC_ID_H264, draw_cb));

    // Allocate the packet buffer.
    uint8_t *buffer = new uint8_t[packet_size];

    if (use_udp) {
      boost::asio::ip::udp::endpoint listen_endpoint(boost::asio::ip::address_v4::any(), port);
      ip::udp::socket sock(io_context, listen_endpoint);
      sock.set_option(boost::asio::socket_base::broadcast(true));

      bool done = false;
      while (!done) {
	boost::asio::ip::udp::endpoint sender_endpoint;
	size_t recv = sock.receive_from(boost::asio::buffer(buffer, packet_size), sender_endpoint);
	if (recv) {
	  done = !dec->decode(buffer, recv);
	}
	done |= check_for_quit();
      }

    } else if (use_srt) {
      int ss, st;
      struct sockaddr_in sa;
      int yes = 1;
      struct sockaddr_storage their_addr;

      // Initialize the SRT library
      srt_startup();

      // Create the SRT socket
      ss = srt_create_socket();
      if (ss == SRT_ERROR) {
	fprintf(stderr, "srt_socket: %s\n", srt_getlasterror_str());
	return 1;
      }

      // Set some socket options
      SRT_TRANSTYPE tt = SRTT_FILE;
      srt_setsockopt(ss, 0, SRTO_TRANSTYPE, &tt, sizeof tt);

      // Ge the port info
      sa.sin_family = AF_INET;
      sa.sin_port = htons(port);
      if (inet_pton(AF_INET, hostname.c_str(), &sa.sin_addr) != 1) {
        return 1;
      }

      srt_setsockflag(ss, SRTO_RCVSYN, &yes, sizeof yes);

      // Bind to the port
      st = srt_bind(ss, (struct sockaddr*)&sa, sizeof sa);
      if (st == SRT_ERROR) {
        fprintf(stderr, "srt_bind: %s\n", srt_getlasterror_str());
        return 1;
      }

      bool done = false;
      while (!done) {

	// Listen for connection requests on this port
	st = srt_listen(ss, 2);
	if (st == SRT_ERROR) {
	  fprintf(stderr, "srt_listen: %s\n", srt_getlasterror_str());
	  return 1;
	}

	// Accept the connection.
	std::cout << "Waiting for a connection" << std::endl;
	int addr_size = sizeof their_addr;
	int their_fd = srt_accept(ss, (struct sockaddr *)&their_addr, &addr_size);
	std::cout << "Received a connection" << std::endl;

	// Process the received packets
	while (!done) {

	  size_t recv = srt_recvmsg(their_fd, reinterpret_cast<char*>(buffer), packet_size);

	  if (recv != SRT_ERROR) {
	    done = !dec->decode(buffer, recv);
	  } else {
	    break;
	  }

	  done |= check_for_quit();
	}
      }

    } else if (port > 0) {

      ip::tcp::acceptor acceptor(io_context, ip::tcp::endpoint(ip::tcp::v4(), port));
      bool done = false;
      while (!done) {
	ip::tcp::socket sock(io_context);
	std::cout << "Waiting for a connection" << std::endl;
	acceptor.accept(sock);
	std::cout << "Received a connection" << std::endl;
	bool done = false;
	while (!done) {
	  boost::system::error_code ec;
	  size_t recv = boost::asio::read(sock, boost::asio::buffer(buffer, packet_size),
					  boost::asio::transfer_at_least(1), ec);
	  if (!ec) {
	    done = !dec->decode(buffer, recv);
	  } else {
	    break;
	  }

	  done |= check_for_quit();
	}
      }

    } else {

      // Wait until we get a connection to a telemetry stream
      while (!telem->connected()) {
	std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      // Try to connect to an RTSP server on the same host.
      boost::asio::ip::tcp::endpoint endpoint(telem->sender_endpoint().address(), 554);
      boost::asio::ip::tcp::socket socket(io_context);
      socket.connect(endpoint);
      socket.close();

      // Connect to the RTSP server and parse the stream
      std::string rtsp_url = "rtsp://" + endpoint.address().to_string() + "/media/stream2";
      dec.reset(new FFMpegDecoder(rtsp_url, draw_cb));

      //while (dec->decode_url() && !check_for_quit()) {}
      while (!g_quit && dec->decode_url()) {}

/*
      uint8_t buf[1024];

      bool done = false;
      while (!done) {
	size_t recv = fread(buf, 1, 1024, stdin);
	if (recv > 0) {
	  done = !dec->decode(buffer, recv);
	}
	done |= check_for_quit();
      }
*/

    }

    draw_thread.join();

    delete [] buffer;
  }

  return 0;
}
