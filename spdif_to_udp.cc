
#include <stdio.h>
#include <stdlib.h>

#include <alsa/asoundlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

extern "C" {
#include <libdcadec/dca_stream.h>
#include <libdcadec/dca_context.h>
#include <libdcadec/common.h>
}

#define WORD_SIZE 4
#define WORDS_PER_FRAME 1024
#define BUF_WORDS WORDS_PER_FRAME
#define BUF_SAMPLES BUF_WORDS
#define WORDS_PER_SAMPLE (BUF_SAMPLES / BUF_WORDS)
#define SAMPLES_PER_FRAME (WORDS_PER_FRAME / 2)
#define OUT_SAMPLE_SIZE 2
#define OUT_CHANNELS 6
#define SAMPLE_SCALE 400
#define AUDIO_DEVICE "iec958:0"
#define SEND_PORT 2468
#define SEND_ADDR "192.168.1.38"
#define SYNC_WORD 0x80017ffe

main (int argc, char *argv[]) {
  int err;
  unsigned int rate = 48000;
  snd_pcm_t *capture_handle;
  snd_pcm_hw_params_t *hw_params;

  // Open the audio device
  if ((err = snd_pcm_open(&capture_handle, AUDIO_DEVICE, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
    fprintf (stderr, "cannot open audio device %s (%s)\n", AUDIO_DEVICE, snd_strerror (err));
    exit (1);
  }

  // Allocate the hardware parameters structure
  if ((err = snd_pcm_hw_params_malloc(&hw_params)) < 0) {
    fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n",
	     snd_strerror (err));
    exit (1);
  }

  // Initialize the hardware parameters structure
  if ((err = snd_pcm_hw_params_any(capture_handle, hw_params)) < 0) {
    fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
	     snd_strerror (err));
    exit (1);
  }

  // Set the access type
  if ((err = snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
    fprintf (stderr, "cannot set access type (%s)\n",
	     snd_strerror (err));
    exit (1);
  }

  // Set the format to signed 16 bit little-endian
  if ((err = snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
    fprintf (stderr, "cannot set sample format (%s)\n",
	     snd_strerror (err));
    exit (1);
  }

  // Set the rate to 48000 sample/second
  if ((err = snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &rate, 0)) < 0) {
    fprintf (stderr, "cannot set sample rate (%s)\n", snd_strerror(err));
    exit (1);
  }

  // Set the number of channels to 2.
  if ((err = snd_pcm_hw_params_set_channels(capture_handle, hw_params, 2)) < 0) {
    fprintf (stderr, "cannot set channel count (%s)\n",
	     snd_strerror (err));
    exit (1);
  }

  // Set the parameters.
  if ((err = snd_pcm_hw_params(capture_handle, hw_params)) < 0) {
    fprintf (stderr, "cannot set parameters (%s)\n",
	     snd_strerror (err));
    exit (1);
  }

  // Free the hardware parameters structure
  snd_pcm_hw_params_free(hw_params);

  // Prepare the audio device for use.
  if ((err = snd_pcm_prepare(capture_handle)) < 0) {
    fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
	     snd_strerror (err));
    exit (1);
  }

  // Create the DTS decoder stream.
  struct dcadec_stream *stream = dcadec_stream_create();

  // Create the DCA decode context
  int flags = DCADEC_FLAG_STRICT;
  struct dcadec_context *context = dcadec_context_create(flags);
  if (!context) {
    fprintf(stderr, "Couldn't create decoder context\n");
    exit(1);
  }

  // Create a send socket
  struct sockaddr_in local;
  local.sin_family = AF_INET;
  local.sin_port = htons(SEND_PORT);
  local.sin_addr.s_addr = inet_addr(SEND_ADDR);
  int socketC = socket(AF_INET, SOCK_DGRAM, 0);

  // Start reading
  uint32_t obuf[WORDS_PER_FRAME];
  uint16_t obuf_idx = 0;
  uint8_t outbuf[SAMPLES_PER_FRAME * OUT_SAMPLE_SIZE * OUT_CHANNELS];
  uint16_t *outbuf16 = reinterpret_cast<uint16_t*>(outbuf);
  bool found_start = false;
  bool found_end = false;
  while (1) {

    // Read a packet from the audio interface.
    uint32_t buf[BUF_WORDS];
    int nread = snd_pcm_readi(capture_handle, (void*)buf, BUF_SAMPLES);
    if (nread > 0) {

      // Send out as many packets as we can.
      for (int buf_idx = 0; buf_idx != BUF_WORDS; ++buf_idx) {

	if (buf[buf_idx] == SYNC_WORD) {
	  if (found_start) {
	    found_end = true;
	    --buf_idx;
	  } else {
	    found_start = true;
	    obuf_idx = 0;
	    obuf[obuf_idx++] = buf[buf_idx];
	  }
	} else {
	  // Copy the data to the output buffer between the start sync and end
	  if (found_start) {
	    // Reject the frame if it's too large.
	    if (obuf_idx < WORDS_PER_FRAME) {
	      obuf[obuf_idx++] = buf[buf_idx];
	    } else {
	      obuf_idx = 0;
	      found_start = false;
	      found_end = false;
	    }
	  }
	}

	// Decode the packet when we have a complete frame
	if (found_end) {

	  // Parse the packet.
	  uint8_t *packet = 0;
	  size_t size = 0;
	  int err = dcadec_stream_parse(stream, (uint8_t*)obuf, obuf_idx * WORD_SIZE,
					&packet, &size);
	  if (err > 0) {
	    // Parse the context
	    if ((err = dcadec_context_parse(context, packet, size)) < 0) {
	      printf("Error parsing packet: %s\n", dcadec_strerror(err));
	    } else {
	      // Extract the samples
	      int **samples = 0;
	      int nsamples = 0;
	      int channel_mask = 0;
	      int sample_rate = 0;
	      int bits_per_sample = 0;
	      if ((err = dcadec_context_filter(context, &samples, &nsamples,
					       &channel_mask, &sample_rate,
					       &bits_per_sample, NULL)) < 0) {
		printf("Error filtering frame: %s\n\r", dcadec_strerror(err));
	      }
	      int nchannels = dca_popcount(channel_mask);

	      // Output the samples.
	      int16_t oidx = 0;
	      int nclipped = 0;
	      for (int i = 0; i < nsamples; i++) {
		for (int j = 0; j < nchannels; j++) {
		  outbuf16[oidx++] = samples[j][i] / SAMPLE_SCALE;
		}
	      }
	      if (sendto(socketC, outbuf, SAMPLES_PER_FRAME * OUT_SAMPLE_SIZE * OUT_CHANNELS,
			 0, (const struct sockaddr*)&local,sizeof(local)) < 0) {
		fprintf(stderr, "Socket send error\n");
	      }
	    }
	  } else {
	    printf("Error parsing stream: %s\n\r", dcadec_strerror(err));
	  }

	  obuf_idx = 0;
	  found_start = false;
	  found_end = false;
	}
      }
    } else {
      // Just wait a second and try again on read errors.
      sleep(1);
    }
  }

  // Cleanup and exit
  close(socketC);
  snd_pcm_close(capture_handle);
  exit(0);
}
