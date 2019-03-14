#include <sys/types.h>
#include <stdio.h>
#include <iostream>

#include <boost/asio.hpp>

#include <RtAudio.h>

#define PACKET_SIZE 6144
#define NCHANNELS 6
#define PORT_NUM 2468
#define SAMPLE_RATE 47999
#define BUFFER_FRAMES 512
#define FORMAT RTAUDIO_SINT16
uint32_t obufidx = 0;

int output(void *out_buffer, void * /*in_buffer*/, unsigned int nframes,
	   double /*stream_time*/, RtAudioStreamStatus /*status*/, void *data) {
#if 0
  SOCKET &socketC = *((SOCKET*)data);
  int buffer_bytes = nframes * NCHANNELS * sizeof(int16_t);

  // Try to read UDP packets
  struct sockaddr_in from;
  int from_len = sizeof(from);
  int rec_len = recvfrom(socketC, (char*)out_buffer, buffer_bytes, 0,
			 (sockaddr*)&from, &from_len);
#endif

  return 0;
}

int main(int argc, char* argv[]) {

  // Ensure we have a valid output device.
  RtAudio dac;
  if (dac.getDeviceCount() < 1) {
    std::cerr << "No audio devices found!" << std::endl;
    exit(0);
  }

#if 0
  // Initialize the socket interface
  WSADATA wsaData;
  WSAStartup(MAKEWORD(2, 2), &wsaData);

  // Create a recieve socket
  struct sockaddr_in local;
  local.sin_family = AF_INET;
  local.sin_port = htons(PORT_NUM);
  local.sin_addr.s_addr = INADDR_ANY;
  SOCKET socketC = socket(AF_INET, SOCK_DGRAM, 0);

  // Set the socket into non-blocking mode.
/*
  u_long iMode = 1; // non-blocking
  ioctlsocket(socketC, FIONBIO, &iMode);
  struct timeval read_timeout;
  read_timeout.tv_sec = 0;
  read_timeout.tv_usec = 10;
  setsockopt(socketC, SOL_SOCKET, SO_RCVTIMEO, (char*)&read_timeout, sizeof(read_timeout));
*/

  // Bind to the socket
  bind(socketC, (sockaddr*)&local, sizeof(local));
#endif

  // Open the output audio device and start it streaming
  RtAudio::StreamParameters oParams;
  oParams.deviceId = dac.getDefaultOutputDevice();
  oParams.nChannels = NCHANNELS;
  oParams.firstChannel = 0;
  unsigned int bufferFrames = BUFFER_FRAMES;
#if 0
  try {
    dac.openStream(&oParams, NULL, FORMAT, SAMPLE_RATE, &bufferFrames, &output, (void *)&socketC);
    dac.startStream();
  }
  catch (RtAudioError& e) {
    std::cout << '\n' << e.getMessage() << '\n' << std::endl;
  }
  while (1) {
    Sleep(10);
  }
#endif

  dac.closeStream();

#if 0
  closesocket(socketC);
#endif

  return 0;
}
