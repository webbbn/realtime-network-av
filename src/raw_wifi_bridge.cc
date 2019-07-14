
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/ether.h>
#include <netpacket/packet.h>
#include <net/if.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <time.h>

#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>
#include <thread>

#include <boost/program_options.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <shared_queue.hh>
#include <raw_socket.hh>
#include <fec.hh>

namespace po=boost::program_options;

static const uint32_t max_packet = 65000;

struct Message {
  Message(size_t max_packet, uint16_t p) : msg(max_packet), port(p) { }
  uint16_t port;
  std::vector<uint8_t> msg;
};

std::string hostname_to_ip(const std::string &hostname) {

  // Try to lookup the host.
  struct hostent *he;
  if ((he = gethostbyname(hostname.c_str())) == NULL) {
    std::cerr << "Error: invalid hostname\n";
    return "";
  }

  struct in_addr **addr_list = (struct in_addr **)he->h_addr_list;
  for(int i = 0; addr_list[i] != NULL; i++) {
    //Return the first one;
    return inet_ntoa(*addr_list[i]);
  }

  return "";
}

int open_udp_socket_for_rx(int port, const std::string hostname = "") {

  // Try to open a UDP socket.
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    std::cerr << "Error opening the UDP receive socket.\n";
    return -1;
  }

  // Set the socket options.
  int optval = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval , sizeof(int));

  // Find to the receive port
  struct sockaddr_in saddr;
  bzero((char *)&saddr, sizeof(saddr));
  saddr.sin_family = AF_INET;
  saddr.sin_port = htons((unsigned short)port);

  // Lookup the IP address from the hostname
  std::string ip;
  if (hostname != "") {
    ip = hostname_to_ip(hostname);
    saddr.sin_addr.s_addr = inet_addr(ip.c_str());
  } else {
    saddr.sin_addr.s_addr = INADDR_ANY;
  }

  if (bind(fd, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
    std::cerr << "Error binding to the UDP receive socket.\n";
    return -1;
  }

  return fd;
}

double cur_time() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return double(t.tv_sec) + double(t.tv_usec) * 1e-6;
}

int main(int argc, const char** argv) {

  uint16_t block_size;
  uint16_t nblocks;
  uint16_t nfec_blocks;
  bool csv;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("debug,D", "print debug messages")
    ("blocks_size,b", po::value<uint16_t>(&block_size)->default_value(1024),
     "the size of the FEC blocks")
    ("nblocks,n", po::value<uint16_t>(&nblocks)->default_value(8),
     "the number of data blockes used in encoding")
    ("nfec_blocks,k", po::value<uint16_t>(&nfec_blocks)->default_value(4),
     "the number of FEC blockes used in encoding")
    ("csv,c", po::bool_switch(&csv), "output CSV log messages")
    ;

  std::string device;
  std::vector<std::string> ports;
  po::options_description pos("Positional");
  pos.add_options()
    ("device", po::value<std::string>(&device), "the wifi device to use")
    ("port", po::value<std::vector<std::string> >(&ports),
     "the input/output UDP port(s) or host:port(s)")
    ;
  po::positional_options_description p;
  p.add("device", 1);
  p.add("port", -1);

  po::options_description all_options("Allowed options");
  all_options.add(desc).add(pos);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
	    options(all_options).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("port")) {
    std::cout << "Usage: options_description [options] <device> <[host:]port> ...\n";
    std::cout << desc;
    return EXIT_SUCCESS;
  }
  bool debug = (vm.count("debug") > 0);

  // Open the raw socket
  RawSendSocket raw_sock;
  if (!raw_sock.add_device(device)) {
    std::cerr << "Error opeing the raw socket for transmiting.\n";
    std::cerr << "  " << raw_sock.error_msg() << std::endl;
    return EXIT_FAILURE;
  }

  // Create the FEC encoder
  std::shared_ptr<FECEncoder> enc;
  if ((block_size > 0) && (nblocks > 0) && (nfec_blocks > 0)) {
    enc.reset(new FECEncoder(nblocks, nfec_blocks, block_size, true));
  }

  // Create a thread to send packets.
  SharedQueue<std::shared_ptr<Message> > queue;
  auto send_th = [&queue, &raw_sock, enc, block_size, csv]() {
    double start = cur_time();
    size_t count = 0;
    size_t pkts = 0;
    size_t max_pkt = 0;

    // Send message out of the send queue
    while(1) {

      // Pull the next packet off the queue
      std::shared_ptr<Message> msg = queue.pop();

      // FEC encode the packet if requested.
      if (enc) {
	enc->encode(msg->msg.data(), msg->msg.size());
	max_pkt = std::max(static_cast<size_t>(msg->msg.size()), max_pkt);
	for (const uint8_t *block : enc->blocks()) {
	  raw_sock.send(block, block_size + 4, msg->port);
	  count += block_size + 4;
	  ++pkts;
	}
      } else {
	raw_sock.send(msg->msg, msg->port);
	count += msg->msg.size();
	max_pkt = std::max(msg->msg.size(), max_pkt);
	++pkts;
      }
      double cur = cur_time();
      double dur = cur - start;
      if (dur > 2.0) {
	if (csv) {
	  std::cout << pkts << "," << count << "," << queue.size() << "," << max_pkt << std::endl;
	} else {
	  std::cerr << " Packets/sec: " << int(pkts / dur)
		    << " Mbps: " << 8e-6 * count / dur
		    << " Queue size: " << queue.size()
		    << " Max packet: " << max_pkt << std::endl;
	}
	start = cur;
	count = pkts = max_pkt = 0;
      }
    }
  };
  std::thread send_thr(send_th);

  // Open the UDP receive sockets.
  std::vector<std::shared_ptr<std::thread> > thrs;
  for (auto &port : ports) {

    // Parse the hostname and port out of the string
    size_t split = port.find(':');
    int cport;
    std::string hostname;
    if (split == std::string::npos) {
      // No hostname found, so just parse the integer port.
      cport = std::atoi(port.c_str());
    } else {
      hostname = port.substr(0, split).c_str();
      cport = std::atoi(port.substr(split + 1, port.length() - split).c_str());
    }
    int udp_sock = open_udp_socket_for_rx(cport, hostname);
    if (udp_sock < 0) {
      std::cerr << "Error opening the UDP socket.\n";
      return EXIT_FAILURE;
    }

    auto uth = [udp_sock, cport, &queue]() {
      while (1) {
	std::shared_ptr<Message> msg(new Message(max_packet, cport));
	size_t count = recv(udp_sock, msg->msg.data(), max_packet, 0);
	if (count > 0) {
	  msg->msg.resize(count);
	  queue.push(msg);
	}
      }
    };

    thrs.push_back(std::shared_ptr<std::thread>(new std::thread(uth)));
  }

  for(auto th : thrs) {
    th->join();
  }
  send_thr.join();

  return EXIT_SUCCESS;
}
