
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

#include <pcap.h>
#include <pcap-bpf.h>

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

typedef struct {
  pcap_t *ppcap;
  int selectable_fd;
  int n80211HeaderLength;


} monitor_interface_t;

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

static int open_raw_sock_tx(const std::string &ifname) {
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

void open_raw_sock_rx(const char *name, int port, monitor_interface_t *interface) {
  struct bpf_program bpfprogram;
  char szProgram[512];
  char szErrbuf[PCAP_ERRBUF_SIZE];

  int port_encoded = (port * 2) + 1;

  // open the interface in pcap
  szErrbuf[0] = '\0';

  interface->ppcap = pcap_open_live(name, 2350, 0, -1, szErrbuf);
  if (interface->ppcap == NULL) {
    fprintf(stderr, "Unable to open %s: %s\n", name, szErrbuf);
    exit(1);
  }
	
  if(pcap_setnonblock(interface->ppcap, 1, szErrbuf) < 0) {
    fprintf(stderr, "Error setting %s to nonblocking mode: %s\n", name, szErrbuf);
  }

  if(pcap_setdirection(interface->ppcap, PCAP_D_IN) < 0) {
    fprintf(stderr, "Error setting %s direction\n", name);
  }

  int nLinkEncap = pcap_datalink(interface->ppcap);

  if (nLinkEncap == DLT_IEEE802_11_RADIO) {
    //			interface->n80211HeaderLength = 0x18; // Use the first 5 bytes as header, first two bytes frametype, next two bytes duration, then port
    // match on data short, data, rts (and port)
    sprintf(szProgram, "(ether[0x00:2] == 0x0801 || ether[0x00:2] == 0x0802 || ether[0x00:4] == 0xb4010000) && ether[0x04:1] == 0x%.2x", port_encoded);
  } else {
    fprintf(stderr, "ERROR: unknown encapsulation on %s! check if monitor mode is supported and enabled\n", name);
    exit(1);
  }

  if (pcap_compile(interface->ppcap, &bpfprogram, szProgram, 1, 0) == -1) {
    puts(szProgram);
    puts(pcap_geterr(interface->ppcap));
    exit(1);
  } else {
    if (pcap_setfilter(interface->ppcap, &bpfprogram) == -1) {
      fprintf(stderr, "%s\n", szProgram);
      fprintf(stderr, "%s\n", pcap_geterr(interface->ppcap));
    } else {
    }
    pcap_freecode(&bpfprogram);
  }

  interface->selectable_fd = pcap_get_selectable_fd(interface->ppcap);
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
    ;

  std::string direction;
  std::string device;
  std::vector<std::string> ports;
  po::options_description pos("Positional");
  pos.add_options()
    ("dir", po::value<std::string>(&direction), "send or recv (receive)")
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

  // Are we sending or receiving?
  bool sending = false;
  if (direction == "send") {
    sending = true;
  } else if (direction != "recv") {
    std::cerr << "Direction must be either \"send\" or \"recv\"\n";
    return EXIT_FAILURE;
  }

  // Open the raw socket
  int sock;
  std::vector<int> udp_socks;
  if (sending) {
    sock = open_raw_sock_tx(device);
    if (sock < 0) {
      std::cerr << "Error opeing the raw socket for transmiting.\n";
      return EXIT_FAILURE;
    }

    // Open the UDP receive sockets.
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
    // Support more than one socket!
    size_t count = recv(udp_socks[0], recv_ptr, max_packet, 0);
    if (count) {
      std::shared_ptr<std::vector<uint8_t> > send_buf(new std::vector<uint8_t>(hdr_len + count));
      memcpy(send_buf->data() + hdr_len, send_buf->data(), count);
      queue.push(send_buf);
    }
  }
}
