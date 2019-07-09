
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
#include <memory>
#include <thread>
#include <set>

#include <boost/program_options.hpp>

#include <stats_accumulator.hh>
#include <shared_queue.hh>
#include <raw_socket.hh>
#include <fec.hh>

typedef SharedQueue<std::shared_ptr<monitor_message_t> > MsgQueue;

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
  namespace po=boost::program_options;
  std::string hostname;
  uint16_t block_size;
  uint16_t nblocks;
  uint16_t nfec_blocks;
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
  RawReceiveSocket raw_sock;
  if (!raw_sock.add_device(device)) {
    std::cerr << "Error opening the raw socket\n";
    std::cerr << "  " << raw_sock.error_msg() << std::endl;
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

  // Initialize the UDP broadcast output address
  struct sockaddr_in s;
  memset(&s, '\0', sizeof(struct sockaddr_in));
  s.sin_family = AF_INET;
  s.sin_port = (in_port_t)htons(5600);
  s.sin_addr.s_addr = inet_addr(broadcast_ip.c_str());

  // Create the packet receive thread
  MsgQueue queue;
  bool done = false;
  auto recv = [&raw_sock, &queue, &done]() {
    double prev_time = cur_time();
    StatsAccumulator<int8_t> rssi_stats;
    RawReceiveStats prev_stats;
    while(!done) {
      std::shared_ptr<monitor_message_t> msg(new monitor_message_t);
      if (raw_sock.receive(*msg)) {
	queue.push(msg);
	rssi_stats.add(msg->rssi);

	double dur = (cur_time() - prev_time);
	if (dur > 2.0) {
	  prev_time = cur_time();
	  const RawReceiveStats &stats = raw_sock.stats();
	  if (prev_stats.packets == 0) {
	    prev_stats = stats;
	  }
	  std::cerr
	    << "Packets: " << stats.packets - prev_stats.packets << " (D:"
	    << stats.dropped_packets - prev_stats.dropped_packets << " E:"
	    << stats.error_packets - prev_stats.error_packets
	    << ")  MB: " << static_cast<float>(stats.bytes) * 1e-6
	    << " (" << static_cast<float>(stats.bytes - prev_stats.bytes) * 1e-6
	    << " - " << 8e-6 * static_cast<double>(stats.bytes - prev_stats.bytes) / dur
	    << " Mbps)  Resets: " << stats.resets << "-" << stats.resets - prev_stats.resets
	    << "  RSSI: " << static_cast<int16_t>(rint(rssi_stats.mean())) << " ("
	    << static_cast<int16_t>(rssi_stats.min()) << "/"
	    << static_cast<int16_t>(rssi_stats.max()) << ")\n";
	  prev_stats = stats;
	  rssi_stats.reset();
	}
      } else {
	std::cerr << "Error receiving packet.\n" << raw_sock.error_msg() << std::endl;
      }
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

    // Is the packet FEC encoded?
    if ((block_size > 0) && (nblocks > 0) && (nfec_blocks > 0)) {

      // Create the FEC decoder if necessary
      if (decoders.find(buf->port) == decoders.end()) {
	decoders[buf->port] =
	  std::shared_ptr<FECDecoder>(new FECDecoder(nblocks, nfec_blocks, block_size, true));
      }
      std::shared_ptr<FECDecoder> fec = decoders[buf->port];

      // Add this block to the FEC decoder.
      if (fec->add_block(buf->data.data(), buf->seq_num) == FECStatus::FEC_COMPLETE) {

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
	    usleep(1000);
	  }
	}
      }

      double dur = (cur_time() - prev_time);
      if (dur > 2.0) {
	// Combine all decoder stats.
	const FECDecoderStats &stats = fec->stats();
	std::cerr
	  << "Decode stats: "
	  << stats.total_packets - prev_stats.total_packets << " / "
	  << stats.dropped_packets - prev_stats.dropped_packets << std::endl;
	prev_time = cur_time();
	prev_stats = stats;
      }

    } else {

      // Just relay the packet if we're not FEC decoding.
      s.sin_port = (in_port_t)htons(buf->port);
      sendto(sock, buf->data.data(), buf->data.size(), 0, (struct sockaddr *)&s,
	     sizeof(struct sockaddr_in));
    }
  }
}
