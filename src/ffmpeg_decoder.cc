#include <iostream>

#include "ffmpeg_decoder.hh"

FFMpegDecoder::FFMpegDecoder(AVCodecID decoder, DrawCallback cb) :
   m_parser(0), m_format_ctx(0), m_sws_ctx(0), m_draw(cb) {

  av_register_all();

  // Find the decoder
  if (!(m_codec = avcodec_find_decoder(decoder))) {
    std::cerr << "Error finding the decoder" << std::endl;
    exit(1);
  }

  // Allocate the context
  if (!(m_codec_ctx = avcodec_alloc_context3(m_codec))) {
    std::cerr << "Error opening the decoding context" << std::endl;
    exit(1);
  }

  // Create the parser
  if (!(m_parser = av_parser_init(m_codec->id))) {
    std::cerr << "Error creating the parser" << std::endl;
    exit(1);
  }

  // Call the common init method
  init();
}

// Initialize a class for decoding from a URL path
FFMpegDecoder::FFMpegDecoder(const std::string &url, DrawCallback cb) :
  m_format_ctx(0), m_sws_ctx(0), m_draw(cb) {

  av_register_all();

  // Register the network interface
  avformat_network_init();

  // Open the input stream.
  AVDictionary *opts = 0;
  av_dict_set(&opts, "rtsp_transport", "tcp", 0);
  if (avformat_open_input(&m_format_ctx, url.c_str(), NULL, &opts) != 0) {
    std::cerr << "Error connecting to the server" << std::endl;
    exit(1);
  }

  // Start reading packets from stream
  av_read_play(m_format_ctx);

  // Get the stream information.
  if (avformat_find_stream_info(m_format_ctx, NULL) < 0) {
    std::cerr << "Error opening stream info" << std::endl;
    exit(1);
  }

  // Find the video stream.
  m_video_stream = av_find_best_stream(m_format_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
  if (m_video_stream < 0) {
    av_log(NULL, AV_LOG_ERROR, "Can't find video stream in input file\n");
    exit(1);
  }

  AVCodecParameters *origin_par = m_format_ctx->streams[m_video_stream]->codecpar;

  // Find the decoder
  if (!(m_codec = avcodec_find_decoder(origin_par->codec_id))) {
    std::cerr << "Error finding the decoder" << std::endl;
    exit(1);
  }

  // Allocate the context
  if (!(m_codec_ctx = avcodec_alloc_context3(m_codec))) {
    std::cerr << "Error opening the decoding context" << std::endl;
    exit(1);
  }

  if (avcodec_parameters_to_context(m_codec_ctx, origin_par)) {
    std::cerr << "Error copying the decoder context" << std::endl;
    exit(1);
  }

  // Call the common init method
  init();
}

FFMpegDecoder::~FFMpegDecoder() {
  if (m_parser) {
    av_parser_close(m_parser);
  }
  avcodec_close(m_codec_ctx);
  av_free(m_frame);
}

bool FFMpegDecoder::decode_packet() {

  // Decode the packet.
  if (m_packet.size) {
    if (avcodec_send_packet(m_codec_ctx, &m_packet) < 0) {
      std::cerr << "Error in avcodec_send_packet" << std::endl;
      return true;
    }

    if(avcodec_receive_frame(m_codec_ctx, m_frame) < 0) {
      return true;
    }

    // Did we decoded a frame parsed out of the stream
    int width = m_codec_ctx->width;
    int height = m_codec_ctx->height;

    // Allocate the conversion context
    if (m_sws_ctx == 0) {

      // initialize SWS context for software scaling
      m_sws_ctx = sws_getContext(width, height, m_codec_ctx->pix_fmt, width, height,
				 AV_PIX_FMT_YUV420P, SWS_BILINEAR, NULL, NULL, NULL);

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
    sws_scale(m_sws_ctx, (uint8_t const * const *)m_frame->data,
	      m_frame->linesize, 0, height, pict.data, pict.linesize);

    // Call the external draw function
    if (m_draw) {
      m_draw(width, height, m_y_plane, m_u_plane, m_v_plane);
    }
  }

  return true;
}

bool FFMpegDecoder::decode(unsigned char *pData, size_t sz) {
  uint8_t *data = pData;
  size_t data_size = sz;
  static bool first = false;
  while (data_size > 0) {
    if (first) {
      first = false;
      return true;
    }

    // Trye to parse a frame out of the buffer
    int consumed = av_parser_parse2(m_parser, m_codec_ctx, &(m_packet.data), &(m_packet.size),
				    data, data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);

    // Move the data pointer passed the consumed bytes
    data += consumed;
    data_size -= consumed;

    // Decode and play the packet.
    if (!decode_packet()) {
      return decode_packet();
    }
  }

  return true;
}

bool FFMpegDecoder::decode_url() {
  if (av_read_frame(m_format_ctx, &m_packet) >= 0) {
    if (m_packet.stream_index == m_video_stream) {
      if (!decode_packet()) {
	return false;
      }
    }
    return true;
  }
  return false;
}

void FFMpegDecoder::init() {

  // Open the codec
  if (avcodec_open2(m_codec_ctx, m_codec, 0) < 0) {
    std::cerr << "Error opening the decoding codec" << std::endl;
    exit(1);
  }

  // Allocate the YUV frame.
  if (!(m_frame = av_frame_alloc())) {
    std::cerr << "Error allocating the frame" << std::endl;
    exit(1);
  }

  // Allocate the packet
  av_init_packet(&m_packet);
}

