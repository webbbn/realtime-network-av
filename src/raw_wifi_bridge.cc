
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>
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

namespace po=boost::program_options;

static const uint32_t max_packet = 65000;

static uint8_t radiotap_header[] = {
  0x00, 0x00, // <-- radiotap version
  0x0c, 0x00, // <- radiotap header length
  0x04, 0x80, 0x00, 0x00, // <-- radiotap present flags (rate + tx flags)
  0x6c, // datarate (will be overwritten later in packet_header_init)
  0x00, // ??
  0x00, 0x00 // ??
};

static uint8_t u8aIeeeHeader_data_short[] = {
  0x08, 0x01, 0x00, 0x00, // frame control field (2bytes), duration (2 bytes)
  0xff // port =  1st byte of IEEE802.11 RA (mac) must be something odd
  // (wifi hardware determines broadcast/multicast through odd/even check)
};

static uint8_t ieee_header_data[] = {
  0x08, 0x02, 0x00, 0x00, // frame control field (2bytes), duration (2 bytes)
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, // port = 1st byte of IEEE802.11 RA (mac) must be something
  // odd (wifi hardware determines broadcast/multicast through odd/even check)
  0x13, 0x22, 0x33, 0x44, 0x55, 0x66, // mac
  0x13, 0x22, 0x33, 0x44, 0x55, 0x66, // mac
  0x00, 0x00 // IEEE802.11 seqnum, (will be overwritten later by Atheros firmware/wifi chip)
};

static uint8_t u8aIeeeHeader_rts[] = {
  0xb4, 0x01, 0x00, 0x00, // frame control field (2 bytes), duration (2 bytes)
  0xff, //  port = 1st byte of IEEE802.11 RA (mac) must be something odd
  // (wifi hardware determines broadcast/multicast through odd/even check)
};

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

static int open_raw_sock(const std::string &ifname) {
  struct sockaddr_ll ll_addr;
  struct ifreq ifr;

  int sock = socket(AF_PACKET, SOCK_RAW, 0);
  if (sock == -1) {
    std::cerr << "Socket open failed.\n";
    return -1;
  }

  ll_addr.sll_family = AF_PACKET;
  ll_addr.sll_protocol = 0;
  ll_addr.sll_halen = ETH_ALEN;

  strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ);

  if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "Error: ioctl(SIOCGIFINDEX) failed\n";
    return -1;
  }

  ll_addr.sll_ifindex = ifr.ifr_ifindex;
  if (ioctl(sock, SIOCGIFHWADDR, &ifr) < 0) {
    std::cerr << "Error: ioctl(SIOCGIFHWADDR) failed\n";
    return -1;
  }

  memcpy(ll_addr.sll_addr, ifr.ifr_hwaddr.sa_data, ETH_ALEN);

  if (bind(sock, (struct sockaddr *)&ll_addr, sizeof(ll_addr)) == -1) {
    std::cerr << "Error: bind failed\n";
    close(sock);
    return -1;
  }

  if (sock == -1) {
    std::cerr <<  "Error: Cannot open socket\n"
      "Info: Must be root with an 802.11 card with RFMON enabled\n";
    return -1;
  }

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 8000;
  if (setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout)) < 0) {
    std::cerr << "setsockopt SO_SNDTIMEO\n";
  }

  int sendbuff = 131072;
  if (setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sendbuff, sizeof(sendbuff)) < 0) {
    std::cerr << "setsockopt SO_SNDBUF\n";
  }

  return sock;
}

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
    ("hostname,H", po::value<std::string>(&hostname), "name or IP address to receive from/send to")
    ;

  std::string device;
  uint32_t port;
  po::options_description pos("Positional");
  pos.add_options()
    ("device", po::value<std::string>(&device), "the wifi device to use")
    ("port", po::value<uint32_t>(&port), "the input/output UDP port")
    ;
  po::positional_options_description p;
  p.add("device", 1);
  p.add("port", 1);

  po::options_description all_options("Allowed options");
  all_options.add(desc).add(pos);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
	    options(all_options).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("device") || !vm.count("port")) {
    std::cout << "Usage: options_description [options] <device> <port>\n";
    std::cout << desc;
    return EXIT_SUCCESS;
  }
  bool debug = (vm.count("debug") > 0);

  // Open the raw socket
  int sock = open_raw_sock(device);
  if (sock < 0) {
    std::cerr << "Error opeing the raw socket.\n";
    return EXIT_FAILURE;
  }

  // Open the UDP receive socket.
  int udp_sock = open_udp_socket_for_rx(port, hostname);
  if (udp_sock < 0) {
    std::cerr << "Error opening the UDP socket.\n";
    return EXIT_FAILURE;
  }

  // Create a thread to send packets.
  Queue<std::shared_ptr<std::vector<uint8_t >> > queue;
  auto send_th = [&queue, sock]() {
    double start = cur_time();
    size_t count = 0;
    size_t pkts = 0;

    // Send message out of the send queue
    while(1) {
      std::shared_ptr<std::vector<uint8_t> > buf = queue.pop();
      send(sock, buf->data(), buf->size(), 0);
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

  // Create the raw packet header + receive buffer.
  uint8_t hdr_len = sizeof(radiotap_header) + sizeof(ieee_header_data);
  std::vector<uint8_t> recv_buf(hdr_len + max_packet);
  memcpy(recv_buf.data(), radiotap_header, sizeof(radiotap_header));
  memcpy(recv_buf.data() + sizeof(radiotap_header), ieee_header_data, sizeof(ieee_header_data));
  uint8_t *recv_ptr = recv_buf.data() + hdr_len;

  // Retrieve messages off the UDP socket.
  while (1) {
    size_t count = recv(udp_sock, recv_ptr, max_packet, 0);
    if (count) {
      std::shared_ptr<std::vector<uint8_t> > send_buf(new std::vector<uint8_t>(hdr_len + count));
      memcpy(send_buf->data() + hdr_len, send_buf->data(), count);
      queue.push(send_buf);
    }
  }
}
