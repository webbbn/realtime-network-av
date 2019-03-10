#ifndef RTNAV_FFMPEG_DECODER_HH
#define RTNAV_FFMPEG_DECODER_HH

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <memory>
#include <functional>

class FFMpegDecoder {
public:
  typedef std::function<void(uint32_t, uint32_t, uint8_t*, uint8_t*, uint8_t*)> DrawCallback;

  // Initialize a class for decoding from memory buffers
  FFMpegDecoder(AVCodecID decoder, DrawCallback cb);

  // Initialize a class for decoding from a URL path
  FFMpegDecoder(const std::string &url, DrawCallback cb);

  ~FFMpegDecoder();

  bool decode_packet();

  bool decode(unsigned char *pData, size_t sz);

  bool decode_url();
  
private:
  AVCodec *m_codec;
  AVCodecContext  *m_codec_ctx;
  AVCodecParserContext *m_parser;
  AVFormatContext *m_format_ctx;
  AVPacket m_packet;
  AVFrame *m_frame;
  int m_video_stream;
  struct SwsContext *m_sws_ctx;
  uint8_t m_font_size;
  uint8_t m_text_border;
  uint8_t *m_y_plane;
  uint8_t *m_u_plane;
  uint8_t *m_v_plane;
  DrawCallback m_draw;

  void init();
};

#endif /* RTNAV_FFMPEG_STREAM_PLAYER_HH */
