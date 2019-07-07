
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

#include <raw_socket.hh>

#include <boost/program_options.hpp>

namespace po=boost::program_options;

static const uint32_t max_packet = 65000;

template <typename tmpl__T>
class Queue {
public:
  Queue() = default;
  ~Queue() = default;

  tmpl__T pop() {
    std::unique_lock<std::mutex> lock_guard(m_mutex);

    while (m_queue.empty()) {
      m_cond.wait(lock_guard);
    }

    auto item = m_queue.front();
    m_queue.pop();
    return item;
  }

  void push(tmpl__T item) {
    std::unique_lock<std::mutex> lock_guard(m_mutex);
    m_queue.push(item);
    lock_guard.unlock();
    m_cond.notify_one();
  }

  size_t size() const {
    return m_queue.size();
  }

private:
  std::queue<tmpl__T> m_queue;
  std::mutex m_mutex;
  std::condition_variable m_cond;
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
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
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

  std::string hostname;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("debug,D", "print debug messages")
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
  p.add("dir", 1);
  p.add("device", 1);
  p.add("port", -11);

  po::options_description all_options("Allowed options");
  all_options.add(desc).add(pos);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
	    options(all_options).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("port")) {
    std::cout << "Usage: options_description [options] <send|recv> <device> <[host:]port> ...\n";
    std::cout << desc;
    return EXIT_SUCCESS;
  }
  bool debug = (vm.count("debug") > 0);

  // Open the raw socket
  RawSendSocket raw_sock;
  if (raw_sock.error()) {
    std::cerr << "Error opeing the raw socket for transmiting.\n";
    std::cerr << "  " << raw_sock.error_msg() << std::endl;
    return EXIT_FAILURE;
  }

  // Open the UDP receive sockets.
  std::vector<int> udp_socks;
  for (auto &port : ports) {
    size_t split = port.find(':');
    int cport;
    std::string hostname;
    if (split == port.length()) {
      cport = std::atoi(port.c_str());
    } else {
      cport = std::atoi(port.substr(0, split).c_str());
      hostname = port.substr(split + 1, port.length() - split).c_str();
    }
    std::cerr << cport << " " << hostname << std::endl;
    int udp_sock = open_udp_socket_for_rx(cport, hostname);
    if (udp_sock < 0) {
      std::cerr << "Error opening the UDP socket.\n";
      return EXIT_FAILURE;
    } else {
      udp_socks.push_back(udp_sock);
    }
  }

  // Create a thread to send packets.
  Queue<std::shared_ptr<std::vector<uint8_t >> > queue;
  auto send_th = [&queue, &raw_sock]() {
    double start = cur_time();
    size_t count = 0;
    size_t pkts = 0;

    // Send message out of the send queue
    while(1) {
      std::shared_ptr<std::vector<uint8_t> > buf = queue.pop();
      raw_sock.send(*buf);
      count += buf->size();
      ++pkts;
      double cur = cur_time();
      double dur = cur - start;
      if (dur > 1.0) {
	std::cerr << pkts << " " << 1e3 * dur / pkts << " " << count << " " << 8e-6 * count / dur
		  << " " << queue.size() << std::endl;
	start = cur;
	count = pkts = 0;
      }
    }
  };
  std::thread send_thr(send_th);

  // Retrieve messages off the UDP socket.
  while (1) {
    // Support more than one socket!
    std::shared_ptr<std::vector<uint8_t> > recv_buf(new std::vector<uint8_t>(max_packet));
    size_t count = recv(udp_socks[0], recv_buf->data(), max_packet, 0);
    if (count) {
      queue.push(recv_buf);
    }
  }
}
