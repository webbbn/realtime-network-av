/*
 * Based on hello_video.c
 * 
 * Copyright (c) 2012, Broadcom Europe Ltd
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Video deocode demo using OpenMAX IL though the ilcient helper library

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <iostream>
#include <thread>

#include "bcm_host.h"
extern "C" {
#include "ilclient.h"
}

bool g_vsync = false;

constexpr uint32_t video_buffer_size = 1 << 18;
constexpr uint8_t num_video_buffers = 2;
uint8_t current_video_read_buffer;
uint8_t current_video_write_buffer;
uint8_t video_buffers[num_video_buffers][video_buffer_size];
uint32_t video_buffer_len[num_video_buffers];

void vsync(DISPMANX_UPDATE_HANDLE_T u, void* arg) {
  g_vsync = true;
}

void read_video() {

  // Try to determine the format based on some key header tags.
  uint32_t nal_count = 0;
  uint32_t tag_count = 0;
  uint32_t search = 0x33333333;
  for (size_t i = 0; i < 1000000; ++i) {
    uint8_t v = fgetc(stdin);
    search = (search << 8) + v;
    if (search == 0x00000167) {
      ++nal_count;
    }
    if (((search %0xffff) == 0x0121) | ((search & 0xffff) == 0x0127)) {
      ++tag_count;
    }
  }
  bool nal_format = (nal_count > tag_count);
  std::cerr << (nal_format ? "NAL format" : "tag format") << std::endl;;

  uint32_t write_len = 0;
  while (1) {

    // Read until we get the next start of a frame or overrunn the buffer.
    //uint16_t search = 0x3333;
    uint32_t search = 0x33333333;
    do {
      // Throw out the frame if we're overrunning the buffer.
      if (write_len == video_buffer_size) {
	search = 0;
	write_len = 0;
	std::cerr << "Read overflow\n";
	continue;
      }
      uint8_t v = fgetc(stdin);
      video_buffers[current_video_write_buffer][write_len++] = v;
      search = (search << 8) + v;
    } while ((nal_format && (search != 0x00000167)) ||
	     (!nal_format && (((search & 0xffff) != 0x0121) &&
			      ((search & 0xffff) != 0x0127))));

    // Don't include the next header in this frame.
    write_len -= (nal_format ? 5 : 2);

    // Store the length so the reader knows this buffer is ready to decode.
    video_buffer_len[current_video_write_buffer] = write_len;

    // We parsed a valid frame. Start writting to the next buffer if we can,
    // or overwrite the current one. Old data is bad data...
    if (write_len > 0) {
      uint8_t next_buffer = (current_video_write_buffer + 1) % num_video_buffers;
      if ((next_buffer != current_video_read_buffer) || (video_buffer_len[next_buffer] == 0)) {
	current_video_write_buffer = next_buffer;
      } else {
	std::cerr << "Discard packet\n";
      }
    }

    // Add the header tag to the beginning of this buffer.
    if (nal_format) {
      uint8_t header[5] = { 0x00, 0x00, 0x00, 0x01, 0x67 };
      memcpy(video_buffers[current_video_write_buffer], header, 5);
      write_len = 5;
    } else {
      video_buffers[current_video_write_buffer][0] = search & 0xff;
      video_buffers[current_video_write_buffer][1] = (search >> 8) & 0xff;
      write_len = 2;
    }
  }
}

int main (int argc, char **argv) {
  bcm_host_init();

  DISPMANX_DISPLAY_HANDLE_T display = vc_dispmanx_display_open(0);
  vc_dispmanx_vsync_callback(display, vsync, NULL);

  COMPONENT_T *list[5]= { 0 };
  int status = 0;

  ILCLIENT_T *client;
  if((client = ilclient_init()) == NULL) {
    return -3;
  }

  if(OMX_Init() != OMX_ErrorNone) {
    ilclient_destroy(client);
    return -4;
  }

  // Start the read thread
  std::thread read_thread(read_video);

  // create video_decode
  COMPONENT_T *video_decode = NULL;
  if(ilclient_create_component(client, &video_decode, const_cast<char*>("video_decode"),
			       static_cast<ILCLIENT_CREATE_FLAGS_T>(ILCLIENT_DISABLE_ALL_PORTS |
								    ILCLIENT_ENABLE_INPUT_BUFFERS))
     != 0) {
    status = -14;
  }
  list[0] = video_decode;

  // create video_render
  COMPONENT_T *video_render = NULL;
  if(status == 0 && ilclient_create_component(client, &video_render,
					      const_cast<char*>("video_render"),
					      ILCLIENT_DISABLE_ALL_PORTS) != 0) {
    status = -14;
  }
  list[1] = video_render;

  TUNNEL_T tunnel[4] = { 0 };
  set_tunnel(tunnel, video_decode, 131, video_render, 90);

  if(status == 0) {
    ilclient_change_component_state(video_decode, OMX_StateIdle);
  }

  OMX_VIDEO_PARAM_PORTFORMATTYPE format;
  memset(&format, 0, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
  format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
  format.nVersion.nVersion = OMX_VERSION;
  format.nPortIndex = 130;
  format.eCompressionFormat = OMX_VIDEO_CodingAVC;

  if((status == 0) &&
     OMX_SetParameter(ILC_GET_HANDLE(video_decode), OMX_IndexParamVideoPortFormat, &format) ==
     OMX_ErrorNone &&
     ilclient_enable_port_buffers(video_decode, 130, NULL, NULL, NULL) == 0) {
    OMX_BUFFERHEADERTYPE *buf;
    bool rendering_started = false;
    int first_packet = 1;

    ilclient_change_component_state(video_decode, OMX_StateExecuting);

    while((buf = ilclient_get_input_buffer(video_decode, 130, 1)) != NULL) {

      do {

	// waiting for vsync
	//g_vsync = false;
	//while (!g_vsync) {
	//usleep(100);
	//}

	// Have we recieved some data to decode?
	uint32_t read_len = video_buffer_len[current_video_read_buffer];
	if (read_len == 0) {
	  usleep(100);
	  continue;
	}

	// Copy the current read buffer to the decode buffer.
	if (buf->nAllocLen >= read_len) {
	  memcpy(buf->pBuffer, video_buffers[current_video_read_buffer], read_len);
	  buf->nFilledLen = read_len;
	} else {
	  std::cerr << "decode overflow " << buf->nAllocLen << " " << read_len << std::endl;
	  // The frame is too large!
	  buf->nFilledLen = 0;
	}
	video_buffer_len[current_video_read_buffer] = 0;
	current_video_read_buffer = (current_video_read_buffer + 1) % num_video_buffers;

	// Start the rendering process when there is decoded video available to render.
	if(!rendering_started &&
	   (((buf->nFilledLen > 0) &&
	     ilclient_remove_event(video_decode, OMX_EventPortSettingsChanged, 131, 0, 0, 1) == 0) ||
	    ((buf->nFilledLen == 0) &&
	     ilclient_wait_for_event(video_decode, OMX_EventPortSettingsChanged,
				     131, 0, 0, 1, ILCLIENT_EVENT_ERROR | ILCLIENT_PARAMETER_CHANGED,
				     10000) == 0))) {
	  rendering_started = true;

	  if(ilclient_setup_tunnel(tunnel, 0, 0) != 0) {
	    status = -7;
	    break;
	  }

	  ilclient_change_component_state(video_render, OMX_StateExecuting);
	}

      } while (buf->nFilledLen == 0);

      // Start decoding the frame
      buf->nOffset = 0;
      if(first_packet) {
	buf->nFlags = OMX_BUFFERFLAG_STARTTIME;
	first_packet = 0;
      } else{
	buf->nFlags = OMX_BUFFERFLAG_TIME_UNKNOWN;
      }
      if(OMX_EmptyThisBuffer(ILC_GET_HANDLE(video_decode), buf) != OMX_ErrorNone) {
	status = -6;
	break;
      }
    }

    buf->nFilledLen = 0;
    buf->nFlags = OMX_BUFFERFLAG_TIME_UNKNOWN | OMX_BUFFERFLAG_EOS;

    if(OMX_EmptyThisBuffer(ILC_GET_HANDLE(video_decode), buf) != OMX_ErrorNone) {
      status = -20;
    }

    // wait for EOS from render
    ilclient_wait_for_event(video_render, OMX_EventBufferFlag, 90, 0, OMX_BUFFERFLAG_EOS, 0,
			    ILCLIENT_BUFFER_FLAG_EOS, -1);

    // need to flush the renderer to allow video_decode to disable its input port
    ilclient_flush_tunnels(tunnel, 0);
  }

  ilclient_disable_tunnel(tunnel);
  ilclient_disable_tunnel(tunnel+1);
  ilclient_disable_tunnel(tunnel+2);
  ilclient_disable_port_buffers(video_decode, 130, NULL, NULL, NULL);
  ilclient_teardown_tunnels(tunnel);

  ilclient_state_transition(list, OMX_StateIdle);
  ilclient_state_transition(list, OMX_StateLoaded);

  ilclient_cleanup_components(list);

  OMX_Deinit();

  ilclient_destroy(client);
  return status;
}
