/* V4L2 video picture grabber
 *
 * Based on code from: https://gist.github.com/lightbits/70399ac79ec005751e94#file-c920-cpp
 *
 * Copyright (C) 2009 Mauro Carvalho Chehab <mchehab@infradead.org>
 * With modifications by Brian Webb <webbbn@gmail.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2 of the License.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * Modified by Ascend NTNU:
 * * H264 streaming for Logitech C920
 * * Commentary
 * * Rewrote from C to C++
 * * Output to single file instead of frame by frame
 * * Releasing on-device allocated memory by calling VIDIOC_REQBUF
 * with count 0
 * * Setting the framerate
 */

// How to compile
// --------------------------------------------------------
// Acquire the video 4 linux 2 development libraries (v4l2)
//   $ sudo apt-get install libv4l-dev
//   $ sudo apt-get install v4l-utils
// Compile with G++
//   $ g++ c920.cpp -lv4l2 -o c920_h264_capture

// Description
// -----------------------------------------------------------
// The code will initiate a H264 stream from the camera device
// located at /dev/video1. It will store N frames concatenated
// in a file named output.raw.

// See https://github.com/bellbind/node-v4l2camera/wiki/V4l2-programming
// for general workflow with the v4l2 libraries.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <libv4l2.h>

#include <iostream>

#include <boost/asio.hpp>

#if BOOST_VERSION < 106600
typedef boost::asio::io_service io_context;
#else
typedef boost::asio::io_context io_context;
#endif

bool g_done;

// Wrapper around v4l2_ioctl for programming the video device,
// that automatically retries the USB request if something
// unintentionally aborted the request.
void xioctl(int fh, int request, void *arg) {
  int r;
  do {
    r = v4l2_ioctl(fh, request, arg);
  } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

  if (r == -1) {
    std::cerr << "USB request failed " << errno << ", " << strerror(errno) << std::endl;
    exit(EXIT_FAILURE);
  }
}

void sig_handler(int s){
  switch (s) {
  case SIGINT:
    g_done = true;
    break;
  default:
    break;
  }
}

// Change these to your liking...
// or modify the program to take them cmd arguments!
#define DEVICE_NAME "/dev/video1"
#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720

// This determines the number of "working" buffers we
// tell the device that it can use. I guess 3 is an OK
// amount? Maybe try less or more if it runs badly.
#define MMAP_BUFFERS 5

int main(int argc, char **argv) {
  namespace ip=boost::asio::ip;
  int port = 62876;

  // Configure the signal handler to close gracefully
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = sig_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Create the boost ASIO context for sending the UDP packets
  io_context io_context;

  // Create the UDP socket
  //ip::udp::socket sock(io_context, ip::udp::endpoint(ip::address_v4::any(), 62876));
  //sock.set_option(boost::asio::socket_base::broadcast(true));
  ip::udp::socket sock(io_context);
  boost::system::error_code bs_error;
  sock.open(ip::udp::v4(), bs_error);
  if (bs_error) {
    std::cerr << "Error opening the UDP socket.\n";
    exit(EXIT_FAILURE);
  }

  // Configure the UDP interfact to broadcast.
  sock.set_option(ip::udp::socket::reuse_address(true));
  sock.set_option(boost::asio::socket_base::broadcast(true));

  // Create the broadcast endpoint to send to.
  ip::udp::endpoint udp_bcast_endpoint(ip::address_v4::broadcast(), port);

  // Open the device
  int fd = v4l2_open(DEVICE_NAME, O_RDWR | O_NONBLOCK, 0);
  if (fd < 0) {
    printf("Failed to open device\n");
    exit(EXIT_FAILURE);
  }

  // Specify the format of the data we want from the camera
  // Run v4l2-ctl --device=/dev/video1 --list-formats on the
  // device to see that sort of pixel formats are supported!
  v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = FRAME_WIDTH;
  fmt.fmt.pix.height = FRAME_HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  xioctl(fd, VIDIOC_S_FMT, &fmt);

  // Set streaming parameters, i.e. frames per second.
  // You'll want to query the device for whether or not it
  // supports setting the frame time, and what possible choices
  // it supports.
  // See http://stackoverflow.com/questions/13981933/v4l2-fcntl-ioctl-vidioc-s-parm-for-setting-fps-and-resolution-of-camera-capture
  v4l2_streamparm parm = {};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.output.timeperframe.numerator = 1;
  parm.parm.output.timeperframe.denominator = 30;
  xioctl(fd, VIDIOC_S_PARM, &parm);

  // Sidenote: Run v4l-info /dev/video1 if you want to see what
  // other stuff that the device supports.

  // Check what format we _actually_ got
  printf("Opened device with format:\n");
  printf("Width: %d\n", fmt.fmt.pix.width);
  printf("Height: %d\n", fmt.fmt.pix.height);
  printf("Pixel format: 0x%x\n", fmt.fmt.pix.pixelformat);

  // Request N buffers that are memory mapped between
  // our application space and the device
  v4l2_requestbuffers request = {};
  request.count = MMAP_BUFFERS;
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;
  xioctl(fd, VIDIOC_REQBUFS, &request);

  int num_buffers = request.count;
  std::cout << "Got " << num_buffers << " buffers\n";

  struct Buffer {
    void *start;
    size_t length;
  };
  Buffer buffers[MMAP_BUFFERS];

  // Find out where each requested buffer is located in memory
  // and map them into our application space
  for (int buffer_index = 0; buffer_index < num_buffers; ++buffer_index) {
    v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = buffer_index;
    xioctl(fd, VIDIOC_QUERYBUF, &buf);
    std::cout << "buffer " << buffer_index << "  size = " << buf.length << std::endl;

    buffers[buffer_index].length = buf.length;
    buffers[buffer_index].start =
      mmap(0 /* start anywhere */,
	   buf.length,
	   PROT_READ | PROT_WRITE /* required */,
	   MAP_SHARED /* recommended */,
	   fd, buf.m.offset);

    if (MAP_FAILED == buffers[buffer_index].start) {
      std::cerr << "mmap failed " << errno << ", " << strerror(errno) << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  // Queue the buffers, i.e. indicate to the device
  // that they are available for writing now.
  for (int i = 0; i < num_buffers; ++i) {
    v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    xioctl(fd, VIDIOC_QBUF, &buf);
  }

  // Start stream
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(fd, VIDIOC_STREAMON, &type);

  // Capture n frames
  for (g_done = false; !g_done; ) {
    // The device will now output data continuously.
    // We will use the FD_ZERO/FD_SET/select mechanisms
    // to wait until there is data available from the
    // device. We can specify how long we should wait,
    // and timeout if we think too much time has passed.

    fd_set fds;
    int r = 0;
    do {
      FD_ZERO(&fds);
      FD_SET(fd, &fds);
      timeval tv;
      tv.tv_sec = 2;
      tv.tv_usec = 0;
      r = select(fd + 1, &fds, NULL, NULL, &tv);
    } while ((r == -1 && (errno = EINTR)));

    if (r == -1) {
      printf("select failed\n");
      exit(EXIT_FAILURE);
    }

    // Data has arrived! Let's "dequeue" a buffer to get its data
    v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    xioctl(fd, VIDIOC_DQBUF, &buf);

    // Now we have gotten the data into one of our buffers.
    // buf.index                -> Which mmap'ed buffer is the data located in
    // buffers[buf.index].start -> Where in memory is the data located
    // buf.bytesused            -> Size of data chunk in bytes

    // Do whatever you want with the stream data here!
    // -----------------------------------------------
    // For now, write buffer to an output file. Since we
    // stream in H264, you'll need something like ffmpeg
    // to decode it if you want to check the output. For
    // example:
    //
    // $ ffmpeg -f h264 -i output.raw -vcodec copy output.mp4

    //fwrite(buffers[buf.index].start, buf.bytesused, 1, stdout);
    std::cerr << "Sending " << buf.bytesused << std::endl;
    //sock.send_to(boost::asio::buffer(buffers[buf.index].start, buf.bytesused),
    //udp_bcast_endpoint);

    // Queue buffer for writing again
    xioctl(fd, VIDIOC_QBUF, &buf);
  }

  std::cout << "Shutting down\n";

  // Turn off stream
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(fd, VIDIOC_STREAMOFF, &type);

  // Unmap buffers
  for (int i = 0; i < num_buffers; ++i) {
    munmap(buffers[i].start, buffers[i].length);
  }

  // Tell the device that it can release memory
  request.count = 0;
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;
  xioctl(fd, VIDIOC_REQBUFS, &request);

  v4l2_close(fd);
}
