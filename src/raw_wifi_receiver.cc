
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

bool open_raw_sock_rx(const std::string &name, int port, monitor_interface_t *interface) {
  struct bpf_program bpfprogram;
  char errbuf[PCAP_ERRBUF_SIZE];

  int port_encoded = (port * 2) + 1;

  // open the interface in pcap
  errbuf[0] = '\0';
  interface->ppcap = pcap_open_live(name.c_str(), 2350, 0, -1, errbuf);
  if (interface->ppcap == NULL) {
    std::cerr << "Unable to open " << name << ": " << errbuf << std::endl;
    return false;
  }

  if(pcap_setnonblock(interface->ppcap, 1, errbuf) < 0) {
    std::cerr << "Error setting " << name << " to nonblocking mode: " << errbuf << std::endl;
    return false;
  }

  if(pcap_setdirection(interface->ppcap, PCAP_D_IN) < 0) {
    std::cerr << "Error setting " << name << " direction\n";
  }

  int nLinkEncap = pcap_datalink(interface->ppcap);

  char filter[512];
  if (nLinkEncap == DLT_IEEE802_11_RADIO) {
    // match on data short, data, rts (and port)
    sprintf(filter, "(ether[0x00:2] == 0x0801 || ether[0x00:2] == 0x0802 || ether[0x00:4] == 0xb4010000) && ether[0x04:1] == 0x%.2x", port_encoded);
  } else {
    std::cerr << "ERROR: unknown encapsulation on " << name
	      << "! check if monitor mode is supported and enabled\n";
    return false;
  }

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
  uint8_t nblocks;
  uint8_t nfec_blocks;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("debug,D", "print debug messages")
    ("blocks_size,B", po::value<uint16_t>(&block_size)->default_value(1024),
     "the size of the FEC blocks")
    ("nblocks,N", po::value<uint8_t>(&nblocks)->default_value(8),
     "the number of data blockes used in encoding")
    ("nfec_blocks,K", po::value<uint8_t>(&nfec_blocks)->default_value(4),
     "the number of FEC blockes used in encoding")
    ;

  std::string device;
  po::options_description pos("Positional");
  pos.add_options()
    ("device", po::value<std::string>(&device), "the wifi device to use")
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
  bool debug = (vm.count("debug") > 0);

  // Create the FEC decoder
  FECDecoder fec(nblocks, nfec_blocks, block_size, true);

  // Open the raw socket
  monitor_interface_t interface;
  if (!open_raw_sock_rx(device, 0, &interface)) {
    std::cerr << "Error opening the raw socket\n";
    return EXIT_FAILURE;
  }

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

  // Retrieve messages from the raw data socket.
  uint32_t error_packets = 0;
  uint32_t good_packets = 0;
  uint32_t total_blocks = 0;
  uint32_t tx_bytes = 0;
  double prev_time = cur_time();
  while (1) {

    // Recieve the next packet
    struct pcap_pkthdr *pcap_packet_header = NULL;
    uint8_t const *pcap_packet_data = NULL;
    int retval = pcap_next_ex(interface.ppcap, &pcap_packet_header, &pcap_packet_data);
    if (retval < 0) {
      std::cerr << "Error receiving from the raw data socket.\n";
      std::cerr << "  " << pcap_geterr(interface.ppcap) << std::endl;
      return EXIT_FAILURE;
    } else if(retval == 0) {
      // Timeout, just continue;
      continue;
    }

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
      break;
    default:
      break;
    }
    pcap_packet_data -= rt_header_len;

    if (pcap_packet_header->len < (rt_header_len + interface.n80211HeaderLength)) {
      std::cerr
	<< "rx ERROR: ppcapheaderlen < u16headerlen+n80211headerlen: pcap_packet_header->len: "
	<< pcap_packet_header->len << std::endl;
      continue;
    }

    int bytes = pcap_packet_header->len - (rt_header_len + interface.n80211HeaderLength);
    if (bytes < 0) {
      std::cerr << "rx ERROR: bytes < 0: bytes: " << bytes << std::endl;
      continue;
    }
    struct ieee80211_radiotap_iterator rti;
    if (ieee80211_radiotap_iterator_init(&rti,(struct ieee80211_radiotap_header *)pcap_packet_data,
					 pcap_packet_header->len) < 0) {
      std::cerr << "rx ERROR: radiotap_iterator_init < 0\n";
      continue;
    }

    //PENUMBRA_RADIOTAP_DATA prd;
    int n;
    while ((n = ieee80211_radiotap_iterator_next(&rti)) == 0) {
      switch (rti.this_arg_index) {
	/* we don't use these radiotap infos right now, disabled
	   case IEEE80211_RADIOTAP_RATE:
	   prd.m_nRate = (*rti.this_arg);
	   break;
	   case IEEE80211_RADIOTAP_CHANNEL:
	   prd.m_nChannel =
	   le16_to_cpu(*((u16 *)rti.this_arg));
	   prd.m_nChannelFlags =
	   le16_to_cpu(*((u16 *)(rti.this_arg + 2)));
	   break;
	   case IEEE80211_RADIOTAP_ANTENNA:
	   prd.m_nAntenna = (*rti.this_arg) + 1;
	*/
	break;
      case IEEE80211_RADIOTAP_FLAGS:
	//prd.m_nRadiotapFlags = *rti.this_arg;
	break;
      case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
#if 0
	dbm_last[adapter_no] = dbm[adapter_no];
	dbm[adapter_no] = (int8_t)(*rti.this_arg);

	if (dbm[adapter_no] > dbm_last[adapter_no]) { // if we have a better signal than last time, ignore
	  dbm[adapter_no] = dbm_last[adapter_no];
	}

	dbm_ts_now[adapter_no] = current_timestamp();
	if (dbm_ts_now[adapter_no] - dbm_ts_prev[adapter_no] > 220) {
	  dbm_ts_prev[adapter_no] = current_timestamp();
	  rx_status->adapter[adapter_no].current_signal_dbm = dbm[adapter_no];
	  dbm[adapter_no] = 99;
	  dbm_last[adapter_no] = 99;
	}
#endif
	break;
      }
    }

    pcap_packet_data += rt_header_len + interface.n80211HeaderLength;

    // Ralink and Atheros both always supply the FCS to userspace, no need to check
    //if (prd.m_nRadiotapFlags & IEEE80211_RADIOTAP_F_FCS)
    //bytes -= 4;

    // TODO: disable checksum handling in process_payload(), not needed since we have fscfail disabled
    int checksum_correct = 1;

    //rx_status->adapter[adapter_no].received_packet_cnt++;
    //	rx_status->adapter[adapter_no].last_update = dbm_ts_now[adapter_no];
    //	fprintf(stderr,"lu[%d]: %lld\n",adapter_no,rx_status->adapter[adapter_no].last_update);
    //	rx_status->adapter[adapter_no].last_update = current_timestamp();
    
    //process_payload(pcap_packet_data, bytes, checksum_correct, block_buffer_list, adapter_no);

    // Add this block to the FEC decoder.
    //++total_blocks;
    switch (fec.add_block(pcap_packet_data)) {
    case FECStatus::FEC_PARTIAL:
      break;
    case FECStatus::FEC_ERROR:
      ++error_packets;
      break;
    case FECStatus::FEC_COMPLETE:
      ++good_packets;

      // Ensure the block lengths are reasonable.
      if (fec.blocks().size() < nblocks) {
	++error_packets;
      } else {

	// Verify that the block lengths seem reasonable.
	bool sp_error = false;
	for (size_t b = 0; !sp_error && (b < nblocks); ++b) {
	  const uint8_t *block = fec.blocks()[b];
	  uint32_t cur_block_size = *reinterpret_cast<const uint32_t*>(block);
	  if (cur_block_size > block_size) {
	    sp_error = true;
	    ++error_packets;
	  }
	}
	
	// Output the data blocks if they look reasonable.
	struct sockaddr_in s;
	memset(&s, '\0', sizeof(struct sockaddr_in));
	s.sin_family = AF_INET;
	s.sin_port = (in_port_t)htons(5600);
	//s.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	s.sin_addr.s_addr = inet_addr("10.42.0.255");
	if (!sp_error) {
	  total_blocks += nblocks;
	  const std::vector<uint8_t*> &blocks = fec.blocks();
	  for (size_t b = 0; b < nblocks; ++b) {
	    const uint8_t *block = blocks[b];
	    uint32_t cur_block_size = *reinterpret_cast<const uint32_t*>(block);
	    if (cur_block_size > 0) {
	      //std::cout.write(reinterpret_cast<const char*>(block + 4), cur_block_size);
	      sendto(sock, block + 4, cur_block_size, 0, (struct sockaddr *)&s,
		     sizeof(struct sockaddr_in));
	    }
	    tx_bytes += cur_block_size;
	  }
	}
      }
    }

    if ((cur_time() - prev_time) > 2.0) {
      std::cerr << "Good: " << good_packets << "  bad: " << error_packets
		<< "  blocks: " << total_blocks << "  Mbps: "
		<< 8e-6 * tx_bytes / (cur_time() - prev_time) << std::endl;
      prev_time = cur_time();
      tx_bytes = 0;
    }
  }

#if 0
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
#endif
}
