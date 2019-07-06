
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
#include <set>

#include <pcap.h>
#include <pcap-bpf.h>

#include <boost/program_options.hpp>

#include <radiotap.h>
#include <fec.hh>

namespace po=boost::program_options;

static const uint32_t max_packet = 65000;

typedef struct {
  pcap_t *ppcap;
  int selectable_fd;
  int n80211HeaderLength;
  uint16_t block_size;
} monitor_interface_t;

struct monitor_message_t {
  monitor_message_t(size_t data_size = 0) :
    data(data_size), port(0), rssi(0), rate(0), channel(0), channel_flag(0), antenna(0),
    radiotap_flags(0) {}
  std::vector<uint8_t> data;
  uint16_t port;
  int8_t rssi;
  uint8_t rate;
  uint16_t channel;
  uint16_t channel_flag;
  uint8_t antenna;
  uint8_t radiotap_flags;
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

typedef Queue<std::shared_ptr<monitor_message_t> > MsgQueue;


bool open_raw_sock_rx(const std::string &name, int port, monitor_interface_t *interface) {
  struct bpf_program bpfprogram;
  char errbuf[PCAP_ERRBUF_SIZE];

  // open the interface in pcap
  errbuf[0] = '\0';
  interface->ppcap = pcap_open_live(name.c_str(), 2350, 0, -1, errbuf);
  if (interface->ppcap == NULL) {
    std::cerr << "Unable to open " << name << ": " << errbuf << std::endl;
    return false;
  }

/*
  if(pcap_setnonblock(interface->ppcap, 1, errbuf) < 0) {
    std::cerr << "Error setting " << name << " to nonblocking mode: " << errbuf << std::endl;
    return false;
  }
*/

  if(pcap_setdirection(interface->ppcap, PCAP_D_IN) < 0) {
    std::cerr << "Error setting " << name << " direction\n";
  }

  int nLinkEncap = pcap_datalink(interface->ppcap);

  if (nLinkEncap != DLT_IEEE802_11_RADIO) {
    std::cerr << "ERROR: unknown encapsulation on " << name
	      << "! check if monitor mode is supported and enabled\n";
    return false;
  }

  // Match the first 4 bytes of the destination address.
  const char *filter= "(ether[10:4] == 0x13223344)";
  if (pcap_compile(interface->ppcap, &bpfprogram, filter, 1, 0) == -1) {
    puts(filter);
    puts(pcap_geterr(interface->ppcap));
    return false;
  } else {
    if (pcap_setfilter(interface->ppcap, &bpfprogram) == -1) {
      fprintf(stderr, "%s\n", filter);
      fprintf(stderr, "%s\n", pcap_geterr(interface->ppcap));
    } else {
    }
    pcap_freecode(&bpfprogram);
  }

  interface->selectable_fd = pcap_get_selectable_fd(interface->ppcap);

  return true;
}

bool receive_packet(monitor_interface_t &interface, MsgQueue &msg_queue) {

  struct pcap_pkthdr *pcap_packet_header = NULL;
  uint8_t const *pcap_packet_data = NULL;
  while (1) {

/*
    // Wait until a packet is available.
    fd_set readset;
    struct timeval to;
    to.tv_sec = 0;
    to.tv_usec = 1e5; // 100ms
    //to.tv_usec = 1e3; // 1ms
    FD_ZERO(&readset);
    FD_SET(interface.selectable_fd, &readset);
    if(select(30, &readset, NULL, NULL, &to) == 0) {
      continue;
    }
    if(!FD_ISSET(interface.selectable_fd, &readset)) {
      continue;
    }
*/

    // Recieve the next packet
    int retval = pcap_next_ex(interface.ppcap, &pcap_packet_header, &pcap_packet_data);
    if (retval < 0) {
      std::cerr << "Error receiving from the raw data socket.\n";
      std::cerr << "  " << pcap_geterr(interface.ppcap) << std::endl;
      continue;
    } else if(retval == 0) {
      // Timeout, just continue;
      continue;
    }

    break;
  }
  std::shared_ptr<monitor_message_t> msg(new monitor_message_t(interface.block_size + 4));

  // fetch radiotap header length from radiotap header (seems to be 36 for Atheros and 18 for Ralink)
  uint16_t rt_header_len = (pcap_packet_data[3] << 8) + pcap_packet_data[2];

  // check for packet type and set headerlen accordingly
  pcap_packet_data += rt_header_len;
  switch (pcap_packet_data[1]) {
  case 0x01: // data short, rts
    interface.n80211HeaderLength = 0x05;
    break;
  case 0x02: // data
    interface.n80211HeaderLength = 0x18;
    msg->port = static_cast<uint16_t>(pcap_packet_data[14] << 8) + pcap_packet_data[15];
    break;
  default:
    break;
  }
  pcap_packet_data -= rt_header_len;

  if (pcap_packet_header->len < uint32_t(rt_header_len + interface.n80211HeaderLength)) {
    std::cerr
      << "rx ERROR: ppcapheaderlen < u16headerlen+n80211headerlen: pcap_packet_header->len: "
      << pcap_packet_header->len << std::endl;
    return false;
  }

  int bytes = pcap_packet_header->len - (rt_header_len + interface.n80211HeaderLength);
  if (bytes < 0) {
    std::cerr << "rx ERROR: bytes < 0: bytes: " << bytes << std::endl;
    return false;
  }
  struct ieee80211_radiotap_iterator rti;
  if (ieee80211_radiotap_iterator_init(&rti,(struct ieee80211_radiotap_header *)pcap_packet_data,
				       pcap_packet_header->len) < 0) {
    std::cerr << "rx ERROR: radiotap_iterator_init < 0\n";
    return false;
  }

  int n;
  while ((n = ieee80211_radiotap_iterator_next(&rti)) == 0) {
    switch (rti.this_arg_index) {
    case IEEE80211_RADIOTAP_RATE:
      msg->rate = (*rti.this_arg);
      break;
    case IEEE80211_RADIOTAP_CHANNEL:
      msg->channel = *((uint16_t *)rti.this_arg);
      msg->channel_flag = *((uint16_t *)(rti.this_arg + 2));
      break;
    case IEEE80211_RADIOTAP_ANTENNA:
      msg->antenna = (*rti.this_arg) + 1;
      break;
    case IEEE80211_RADIOTAP_FLAGS:
      msg->radiotap_flags = *rti.this_arg;
      break;
    case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
      msg->rssi = (int8_t)(*rti.this_arg);
      break;
    }
  }

  // Copy the data into the message buffer.
  pcap_packet_data += rt_header_len + interface.n80211HeaderLength;
  std::copy(pcap_packet_data, pcap_packet_data + interface.block_size + 4, msg->data.begin());

  // Pass on the message
  msg_queue.push(msg);

  return true;
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

double cur_time() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return double(t.tv_sec) + double(t.tv_usec) * 1e-6;
}

int main(int argc, const char** argv) {

  std::string hostname;
  uint16_t block_size;
  uint16_t nblocks;
  uint16_t nfec_blocks;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("debug,D", "print debug messages")
    ("blocks_size,B", po::value<uint16_t>(&block_size)->default_value(1024),
     "the size of the FEC blocks")
    ("nblocks,N", po::value<uint16_t>(&nblocks)->default_value(8),
     "the number of data blockes used in encoding")
    ("nfec_blocks,K", po::value<uint16_t>(&nfec_blocks)->default_value(4),
     "the number of FEC blockes used in encoding")
    ;

  std::string device;
  std::string broadcast_ip;
  po::options_description pos("Positional");
  pos.add_options()
    ("device", po::value<std::string>(&device), "the wifi device to use")
    ("broadcast_ip", po::value<std::string>(&broadcast_ip),
     "the IP address of the local broadcast port (XX.XX.XX.255)")
    ;
  po::positional_options_description p;
  p.add("device", 1);
  p.add("broadcast_ip", 1);

  po::options_description all_options("Allowed options");
  all_options.add(desc).add(pos);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
	    options(all_options).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("device") || !vm.count("broadcast_ip")) {
    std::cout << "Usage: options_description [options] <device> <broadcast IP>\n";
    std::cout << desc;
    return EXIT_SUCCESS;
  }
  bool debug = (vm.count("debug") > 0);

  // Open the raw socket
  monitor_interface_t interface;
  if (!open_raw_sock_rx(device, 0, &interface)) {
    std::cerr << "Error opening the raw socket\n";
    return EXIT_FAILURE;
  }
  interface.block_size = block_size;

  // Open the UDP send socket
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cerr << "Error opening the UDP send socket.\n";
    return EXIT_FAILURE;
  }
  int trueflag = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &trueflag, sizeof(trueflag)) < 0) {
    std::cerr << "Error setting the UDP send socket to broadcast.\n";
    return EXIT_FAILURE;
  }

  // Initialize the UDP broadcast output address
  struct sockaddr_in s;
  memset(&s, '\0', sizeof(struct sockaddr_in));
  s.sin_family = AF_INET;
  s.sin_port = (in_port_t)htons(5600);
  std::cerr << broadcast_ip << std::endl;
  s.sin_addr.s_addr = inet_addr(broadcast_ip.c_str());

  // Create the packet receive thread
  MsgQueue queue;
  bool done = false;
  auto recv = [&interface, &queue, &done]() {
		while(!done) {
		  receive_packet(interface, queue);
		}
	      };
  std::thread recv_thread(recv);

  // Retrieve messages from the raw data socket.
  double prev_time = cur_time();
  FECDecoderStats prev_stats;
  int8_t rssi_min = 127;
  int8_t rssi_max = -127;
  uint32_t rssi_count = 0;
  float rssi_sum = 0;
  std::map<uint16_t, std::shared_ptr<FECDecoder> > decoders;
  while (!done) {

    // Ralink and Atheros both always supply the FCS to userspace, no need to check
    //if (prd.m_nRadiotapFlags & IEEE80211_RADIOTAP_F_FCS)
    //bytes -= 4;

    //rx_status->adapter[adapter_no].received_packet_cnt++;
    //	rx_status->adapter[adapter_no].last_update = dbm_ts_now[adapter_no];
    //	fprintf(stderr,"lu[%d]: %lld\n",adapter_no,rx_status->adapter[adapter_no].last_update);
    //	rx_status->adapter[adapter_no].last_update = current_timestamp();

    // Pull the next block off the message queue.
    std::shared_ptr<monitor_message_t> buf = queue.pop();
    rssi_min = std::min(buf->rssi, rssi_min);
    rssi_max = std::max(buf->rssi, rssi_max);
    rssi_sum += buf->rssi;
    ++rssi_count;

    // Create the FEC decoder if necessary
    if (decoders.find(buf->port) == decoders.end()) {
      decoders[buf->port] =
	std::shared_ptr<FECDecoder>(new FECDecoder(nblocks, nfec_blocks, block_size, true));
    }
    std::shared_ptr<FECDecoder> fec = decoders[buf->port];

    // Add this block to the FEC decoder.
    if (fec->add_block(buf->data.data()) == FECStatus::FEC_COMPLETE) {

      // Output the data blocks
      const std::vector<uint8_t*> &blocks = fec->blocks();
      for (size_t b = 0; b < nblocks; ++b) {
	const uint8_t *block = blocks[b];
	uint32_t cur_block_size = *reinterpret_cast<const uint32_t*>(block);
	if (cur_block_size > 0) {
	  //std::cout.write(reinterpret_cast<const char*>(block + 4), cur_block_size);
	  s.sin_port = (in_port_t)htons(buf->port);
	  sendto(sock, block + 4, cur_block_size, 0, (struct sockaddr *)&s,
		 sizeof(struct sockaddr_in));
	}
      }
    }

    double dur = (cur_time() - prev_time);
    if (dur > 2.0) {
      // Combine all decoder stats.
      const FECDecoderStats &stats = fec->stats();
      std::cerr << "Blocks: " << stats.total_blocks << "/" << stats.dropped_blocks << "-"
		<< stats.total_blocks - prev_stats.total_blocks << "/"
		<< stats.dropped_blocks - prev_stats.dropped_blocks
		<< "  Packets: " << stats.total_packets << "/" << stats.dropped_packets << "-"
		<< stats.total_packets - prev_stats.total_packets << "/"
		<< stats.dropped_packets - prev_stats.dropped_packets 
		<< "  Bytes: " << stats.bytes << "-" << stats.bytes - prev_stats.bytes
		<< " (" << 8e-6 * static_cast<double>(stats.bytes - prev_stats.bytes) / dur
		<< "Mbps)  Resets: " << stats.lost_sync << "-"
		<< stats.lost_sync - prev_stats.lost_sync
		<< "  RSSI: " << static_cast<int16_t>(rint(rssi_sum / rssi_count))
		<< " (" << static_cast<int16_t>(rssi_min) << "/"
		<< static_cast<int16_t>(rssi_max) << ")\n";
      prev_time = cur_time();
      prev_stats = stats;
      rssi_min = 127;
      rssi_max = -127;
      rssi_sum = 0;
      rssi_count = 0;
    }
  }
}
