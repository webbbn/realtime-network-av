/*
 * Copyright (c) 2015 Ludmila Glinskih
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * H264 codec test.
 */

extern "C" {
#define __STDC_FORMAT_MACROS 1
#include "libavutil/adler32.h"
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/imgutils.h"
#include "libavutil/timestamp.h"
}

static int video_decode_example(const char *input_filename)
{
    AVCodec *codec = NULL;
    AVCodecContext *ctx= NULL;
    AVCodecParameters *origin_par = NULL;
    AVFrame *fr = NULL;
    uint8_t *byte_buffer = NULL;
    AVPacket pkt;
    AVFormatContext *fmt_ctx = NULL;
    int number_of_written_bytes;
    int video_stream;
    int got_frame = 0;
    int byte_buffer_size;
    int i = 0;
    int result;
    int end_of_stream = 0;
    printf("%s\n", input_filename); fflush(stdout);

    result = avformat_open_input(&fmt_ctx, input_filename, NULL, NULL);
    if (result < 0) {
        av_log(NULL, AV_LOG_ERROR, "Can't open file\n");
        return result;
    }
    printf("opened\n"); fflush(stdout);

    result = avformat_find_stream_info(fmt_ctx, NULL);
    if (result < 0) {
        av_log(NULL, AV_LOG_ERROR, "Can't get stream info\n");
        return result;
    }
    printf("abc\n"); fflush(stdout);

    video_stream = av_find_best_stream(fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    if (video_stream < 0) {
      av_log(NULL, AV_LOG_ERROR, "Can't find video stream in input file\n");
      return -1;
    }
    printf("abd\n"); fflush(stdout);

    origin_par = fmt_ctx->streams[video_stream]->codecpar;

    codec = avcodec_find_decoder(origin_par->codec_id);
    if (!codec) {
        av_log(NULL, AV_LOG_ERROR, "Can't find decoder\n");
        return -1;
    }
    printf("abe\n"); fflush(stdout);

    ctx = avcodec_alloc_context3(codec);
    if (!ctx) {
        av_log(NULL, AV_LOG_ERROR, "Can't allocate decoder context\n");
        return AVERROR(ENOMEM);
    }

    result = avcodec_parameters_to_context(ctx, origin_par);
    if (result) {
        av_log(NULL, AV_LOG_ERROR, "Can't copy decoder context\n");
        return result;
    }
    printf("abf\n"); fflush(stdout);

    result = avcodec_open2(ctx, codec, NULL);
    if (result < 0) {
        av_log(ctx, AV_LOG_ERROR, "Can't open decoder\n");
        return result;
    }

    fr = av_frame_alloc();
    if (!fr) {
        av_log(NULL, AV_LOG_ERROR, "Can't allocate frame\n");
        return AVERROR(ENOMEM);
    }

    byte_buffer_size = av_image_get_buffer_size(ctx->pix_fmt, ctx->width, ctx->height, 16);
    byte_buffer = reinterpret_cast<uint8_t*>(av_malloc(byte_buffer_size));
    if (!byte_buffer) {
      av_log(NULL, AV_LOG_ERROR, "Can't allocate buffer\n");
      return AVERROR(ENOMEM);
    }
    printf("abg\n"); fflush(stdout);

    printf("#tb %d: %d/%d\n", video_stream, fmt_ctx->streams[video_stream]->time_base.num, fmt_ctx->streams[video_stream]->time_base.den);
    i = 0;
    av_init_packet(&pkt);
    printf("abh\n"); fflush(stdout);
    do {
      if (!end_of_stream)
	if (av_read_frame(fmt_ctx, &pkt) < 0)
	  end_of_stream = 1;
      if (end_of_stream) {
	pkt.data = NULL;
	pkt.size = 0;
      }
      printf("abi\n"); fflush(stdout);
      if (pkt.stream_index == video_stream || end_of_stream) {
	printf("abj\n"); fflush(stdout);
	got_frame = 0;
	if (pkt.pts == AV_NOPTS_VALUE)
	  pkt.pts = pkt.dts = i;
	result = avcodec_decode_video2(ctx, fr, &got_frame, &pkt);
	if (result < 0) {
	  av_log(NULL, AV_LOG_ERROR, "Error decoding frame\n");
	  return result;
	}
	printf("abk\n"); fflush(stdout);
	if (got_frame) {
	  printf("abl\n"); fflush(stdout);
	  number_of_written_bytes = av_image_copy_to_buffer(byte_buffer, byte_buffer_size,
							    (const uint8_t* const *)fr->data, (const int*) fr->linesize,
							    ctx->pix_fmt, ctx->width, ctx->height, 1);
	  if (number_of_written_bytes < 0) {
	    av_log(NULL, AV_LOG_ERROR, "Can't copy image to buffer\n");
	    return number_of_written_bytes;
	  }
	  printf("Abc\n"); fflush(stdout);
	  /*
	    printf("%d, %s, %s, %8"PRId64", %8d, 0x%08lx\n", video_stream,
	    av_ts2str(fr->pts), av_ts2str(fr->pkt_dts), fr->pkt_duration,
	    number_of_written_bytes, av_adler32_update(0, (const uint8_t*)byte_buffer, number_of_written_bytes));
	  */
	}
	av_packet_unref(&pkt);
	av_init_packet(&pkt);
      }
      i++;
    } while (!end_of_stream || got_frame);

    av_packet_unref(&pkt);
    av_frame_free(&fr);
    avcodec_close(ctx);
    avformat_close_input(&fmt_ctx);
    avcodec_free_context(&ctx);
    av_freep(&byte_buffer);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        av_log(NULL, AV_LOG_ERROR, "Incorrect input\n");
        return 1;
    }

    if (video_decode_example(argv[1]) != 0)
        return 1;

    return 0;
}

#if 0
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libswscale/swscale.h>
}

int main(int argc, char** argv) {

  // Open the initial context variables that are needed
  AVFormatContext* format_ctx = avformat_alloc_context();

  // Register everything
  avformat_network_init();

  // Open RTSP
  if (avformat_open_input(&format_ctx, "rtsp://192.168.1.99/media/stream2",
			  NULL, NULL) != 0) {
    return EXIT_FAILURE;
  }

  AVPacket* packet = av_packet_alloc();

  //start reading packets from stream and write them to file
  av_read_play(format_ctx);    //play RTSP

  // Get the codec
  AVCodec *codec = NULL;
  codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec) {
    exit(1);
  }

  AVCodecParserContext *m_parser;
  if (!(m_parser = av_parser_init(codec->id))) {
    std::cerr << "Error creating the parser" << std::endl;
    exit(1);
  }

  // Allocate teh context for this codec.
  AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);
  if (!codec_ctx) {
    fprintf(stderr, "Could not allocate video codec context\n");
    exit(1);
  }

  std::cerr << "aaa" << std::endl;
  if (avcodec_open2(codec_ctx, codec, 0) < 0) {
    exit(1);
  }
  std::cerr << "bbb" << std::endl;

  AVFrame *m_pFrame = av_frame_alloc();
  while (av_read_frame(format_ctx, packet) >= 0) {
    avcodec_send_packet(codec_ctx, packet);
    avcodec_receive_frame(codec_ctx, m_pFrame);
    std::cerr << "AAA\n";
  }
  std::cerr << "ccc" << std::endl;

  return (EXIT_SUCCESS);
}
#endif
#if 0
/*
 * Copyright (c) 2001 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/**
 * @file
 * video decoding with libavcodec API example
 *
 * @example decode_video.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
#include <libavcodec/avcodec.h>
}

#define INBUF_SIZE 4096

static void pgm_save(unsigned char *buf, int wrap, int xsize, int ysize,
                     char *filename)
{
  FILE *f;
  int i;
  f = fopen(filename,"w");
  fprintf(f, "P5\n%d %d\n%d\n", xsize, ysize, 255);
  for (i = 0; i < ysize; i++)
    fwrite(buf + i * wrap, 1, xsize, f);
  fclose(f);
}
static void decode(AVCodecContext *dec_ctx, AVFrame *frame, AVPacket *pkt,
                   const char *filename)
{
  char buf[1024];
  int ret;
  ret = avcodec_send_packet(dec_ctx, pkt);
  if (ret < 0) {
    fprintf(stderr, "Error sending a packet for decoding\n");
    exit(1);
  }
  while (ret >= 0) {
    ret = avcodec_receive_frame(dec_ctx, frame);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
      return;
    else if (ret < 0) {
      fprintf(stderr, "Error during decoding\n");
      exit(1);
    }
    printf("saving frame %3d\n", dec_ctx->frame_number);
    fflush(stdout);
    /* the picture is allocated by the decoder. no need to
       free it */
    snprintf(buf, sizeof(buf), "%s-%d", filename, dec_ctx->frame_number);
    pgm_save(frame->data[0], frame->linesize[0],
	     frame->width, frame->height, buf);
  }
}
int main(int argc, char **argv)
{
  const char *filename, *outfilename;
  const AVCodec *codec;
  AVCodecParserContext *parser;
  AVCodecContext *c= NULL;
  FILE *f;
  AVFrame *frame;
  uint8_t inbuf[INBUF_SIZE + AV_INPUT_BUFFER_PADDING_SIZE];
  uint8_t *data;
  size_t   data_size;
  int ret;
  AVPacket *pkt;
  if (argc <= 2) {
    fprintf(stderr, "Usage: %s <input file> <output file>\n", argv[0]);
    exit(0);
  }
  filename    = argv[1];
  outfilename = argv[2];
  pkt = av_packet_alloc();
  if (!pkt)
    exit(1);
  /* set end of buffer to 0 (this ensures that no overreading happens for damaged MPEG streams) */
  memset(inbuf + INBUF_SIZE, 0, AV_INPUT_BUFFER_PADDING_SIZE);
  /* find the MPEG-1 video decoder */
  codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec) {
    fprintf(stderr, "Codec not found\n");
    exit(1);
  }
  parser = av_parser_init(codec->id);
  if (!parser) {
    fprintf(stderr, "parser not found\n");
    exit(1);
  }
  c = avcodec_alloc_context3(codec);
  if (!c) {
    fprintf(stderr, "Could not allocate video codec context\n");
    exit(1);
  }
  /* For some codecs, such as msmpeg4 and mpeg4, width and height
     MUST be initialized there because this information is not
     available in the bitstream. */
  /* open it */
  if (avcodec_open2(c, codec, NULL) < 0) {
    fprintf(stderr, "Could not open codec\n");
    exit(1);
  }
  f = fopen(filename, "rb");
  if (!f) {
    fprintf(stderr, "Could not open %s\n", filename);
    exit(1);
  }
  frame = av_frame_alloc();
  if (!frame) {
    fprintf(stderr, "Could not allocate video frame\n");
    exit(1);
  }
  while (!feof(f)) {
    /* read raw data from the input file */
    data_size = fread(inbuf, 1, INBUF_SIZE, f);
    if (!data_size)
      break;
    /* use the parser to split the data into frames */
    data = inbuf;
    while (data_size > 0) {
      ret = av_parser_parse2(parser, c, &pkt->data, &pkt->size,
			     data, data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
      if (ret < 0) {
	fprintf(stderr, "Error while parsing\n");
	exit(1);
      }
      data      += ret;
      data_size -= ret;
      if (pkt->size)
	decode(c, frame, pkt, outfilename);
    }
  }
  /* flush the decoder */
  decode(c, frame, NULL, outfilename);
  fclose(f);
  av_parser_close(parser);
  avcodec_free_context(&c);
  av_frame_free(&frame);
  av_packet_free(&pkt);
  return 0;
}
#endif
