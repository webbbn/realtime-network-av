
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm-generic/termbits.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>

#include <boost/program_options.hpp>

#define CSRF_DEVICE_TYPE 0xc8
#define CSRF_FRAME_LEN 24
#define CSRF_FRAME_TYPE 0x16

typedef enum {
	      DEVICE_ADDRESS,
	      FRAME_LENGTH,
	      FRAME_TYPE,
	      PAYLOAD,
	      CRC
} CSRFStates;

struct CSRFChannels {
    uint16_t ch0 : 11;
    uint16_t ch1 : 11;
    uint16_t ch2 : 11;
    uint16_t ch3 : 11;
    uint16_t ch4 : 11;
    uint16_t ch5 : 11;
    uint16_t ch6 : 11;
    uint16_t ch7 : 11;
    uint16_t ch8 : 11;
    uint16_t ch9 : 11;
    uint16_t ch10 : 11;
    uint16_t ch11 : 11;
    uint16_t ch12 : 11;
    uint16_t ch13 : 11;
    uint16_t ch14 : 11;
    uint16_t ch15 : 11;
} __attribute__ ((__packed__));

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
  crc ^= a;
  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0xD5;
    } else {
      crc = crc << 1;
    }
  }
  return crc;
}

uint8_t crsf_crc(const uint8_t *buf, uint8_t len) {
  uint8_t crc = crc8_dvb_s2(0, CSRF_FRAME_TYPE);
  for(size_t i = 0; i < len; ++i) {
    crc = crc8_dvb_s2(crc, buf[i]);
  }
  return crc;
}

double cur_time() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return 1000 * (double(t.tv_sec) + double(t.tv_usec) * 1e-6);
}

int main(int argc, char **argv) {
  namespace po=boost::program_options;
  uint16_t period;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("period,p", po::value<uint16_t>(&period)->default_value(10),
     "the time in milliseconds between printing successive receiver message")
    ;

  std::string device;
  po::options_description pos("Positional");
  pos.add_options()
    ("device", po::value<std::string>(&device), "the uart device to read from")
    ;
  po::positional_options_description p;
  p.add("device", 1);

  po::options_description all_options("Allowed options");
  all_options.add(desc).add(pos);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
	    options(all_options).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("device")) {
    std::cout << "Usage: options_description [options] <device>\n";
    std::cout << desc;
    return EXIT_SUCCESS;
  }

  // open the serial port
  int device_fd = open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (-1 == device_fd) {
    std::cerr << "Error opening the serial port: " << device << std::endl;
    return EXIT_FAILURE;
  }

  // Get the current UART settings.
  struct termios2 tio { };
  if (0 != ioctl(device_fd, TCGETS2, &tio)) {
    close(device_fd);
    return EXIT_FAILURE;
  }

  // Setting serial port, 8N1, non-blocking.
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;

  // Use BOTHER to specify speed directly in c_[io]speed member
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = 420000;
  tio.c_ospeed = 420000;
  tio.c_cc[VMIN] = 25;
  tio.c_cc[VTIME] = 0;

  // Change the UART settings.
  if (0 != ioctl(device_fd, TCSETS2, &tio)) {
    close(device_fd);
    return EXIT_FAILURE;
  }

  // Parse the Crossfire receiver messages and print the value preiodically
  uint8_t stage = DEVICE_ADDRESS;
  bool reset = false;
  CSRFChannels channels;
  uint8_t *cbuf = reinterpret_cast<uint8_t*>(&channels);
  uint8_t idx = 0;
  double prev_time = cur_time();
  while(1) {
    uint8_t buf[1024];
    int count = read(device_fd, buf, 1024);
    for (int i = 0; i < count; ++i) {
      uint16_t c = buf[i];
      switch(stage) {
      case DEVICE_ADDRESS:
	// The first byte of  a frame is the device type
	if (c == CSRF_DEVICE_TYPE) {
	  stage = FRAME_LENGTH;
	}
	break;
      case FRAME_LENGTH:
	// The second byte is the frame length
	if (c == CSRF_FRAME_LEN) {
	  stage = FRAME_TYPE;
	} else {
	  reset = true;
	}
	break;
      case FRAME_TYPE:
	// The frame type is the third byte
	if (c == CSRF_FRAME_TYPE) {
	  stage = PAYLOAD;
	} else {
	  reset = true;
	}
	break;
      case PAYLOAD:
	// Extract the bits into the payload buffer
	cbuf[idx++] = c;
	if(idx == sizeof(channels)) {
	  stage = CRC;
	}
	break;
      case CRC:
	uint8_t crc = crsf_crc(cbuf, sizeof(channels));
	if (crc != c) {
	  printf("error: %x %x\n", crc, c);
	}
	double time = cur_time();
	if ((time - prev_time) >= period) {
	  printf("rx,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		 channels.ch0, channels.ch1, channels.ch2, channels.ch3,
		 channels.ch4, channels.ch5, channels.ch6, channels.ch7,
		 channels.ch8, channels.ch9, channels.ch10, channels.ch11,
		 channels.ch12, channels.ch13, channels.ch14, channels.ch15);
	  prev_time = time;
	}
	reset = true;
	break;
      }

      // Reset if requested
      if (reset) {
	stage = DEVICE_ADDRESS;
	idx = 0;
	reset = false;
      }
    }
  }
}
