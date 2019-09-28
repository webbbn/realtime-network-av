
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/ether.h>
#include <netpacket/packet.h>
#include <net/if.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <time.h>
#include <ifaddrs.h>

#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>
#include <thread>
#include <set>

#include <boost/program_options.hpp>

#include <boost/foreach.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <stats_accumulator.hh>
#include <shared_queue.hh>
#include <raw_socket.hh>
#include <fec.hh>

namespace po=boost::program_options;
namespace pt=boost::property_tree;

std::string hostname_to_ip(const std::string &hostname);

struct WifiOptions {
  WifiOptions(LinkType type = DATA_LINK, uint8_t rate = 18, bool m = false, bool s = false, bool l = false) :
    link_type(type), data_rate(rate), mcs(m), stbc(s), ldpc(l) { }
  LinkType link_type;
  uint8_t data_rate;
  bool mcs;
  bool stbc;
  bool ldpc;
};

struct Message {
  Message(size_t max_packet, uint8_t p, uint8_t pri, WifiOptions opt,
	  std::shared_ptr<FECEncoder> e) :
    msg(max_packet), port(p), priority(pri), opts(opt), enc(e) { }
  std::vector<uint8_t> msg;
  uint8_t port;
  uint8_t priority;
  WifiOptions opts;
  std::shared_ptr<FECEncoder> enc;
};

struct UDPDestination {
  UDPDestination(uint16_t port, const std::string &hostname, std::shared_ptr<FECDecoder> enc) :
    fec(enc) {

    // Initialize the UDP output socket.
    memset(&s, '\0', sizeof(struct sockaddr_in));
    s.sin_family = AF_INET;
    s.sin_port = (in_port_t)htons(port);

    // Lookup the IP address from the hostname
    std::string ip;
    if (hostname != "") {
      ip = hostname_to_ip(hostname);
      s.sin_addr.s_addr = inet_addr(ip.c_str());
    } else {
      s.sin_addr.s_addr = INADDR_ANY;
    }
    s.sin_addr.s_addr = inet_addr(ip.c_str());
  }
  struct sockaddr_in s;
  std::shared_ptr<FECDecoder> fec;
};

bool get_wifi_devices(std::vector<std::string> &ifnames) {
  int family, s, n;
  char host[NI_MAXHOST];

  // Get the wifi interfaces.
  struct ifaddrs *ifaddr;
  if (getifaddrs(&ifaddr) == -1) {
    return true;
  }

  // Walk through linked list, maintaining head pointer so we can free list later
  struct ifaddrs *ifa;
  for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
    if (ifa->ifa_addr == NULL) {
      continue;
    }

    int family = ifa->ifa_addr->sa_family;

    // Display interface name and family (including symbolic form of the latter for the common families)
    printf("%-8s %s (%d)\n",
	   ifa->ifa_name,
	   (family == AF_PACKET) ? "AF_PACKET" :
	   (family == AF_INET) ? "AF_INET" :
	   (family == AF_INET6) ? "AF_INET6" : "???",
	   family);

    // For an AF_INET* interface address, display the address
    if (family == AF_INET || family == AF_INET6) {
      s = getnameinfo(ifa->ifa_addr,
		      (family == AF_INET) ? sizeof(struct sockaddr_in) :
		      sizeof(struct sockaddr_in6),
		      host, NI_MAXHOST,
		      NULL, 0, NI_NUMERICHOST);
      if (s != 0) {
	printf("getnameinfo() failed: %s\n", gai_strerror(s));
	exit(EXIT_FAILURE);
      }

      printf("\t\taddress: <%s>\n", host);

    }
  }

  freeifaddrs(ifaddr);
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

int open_udp_socket_for_rx(uint16_t port, const std::string hostname = "") {

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
  saddr.sin_port = htons(port);

  // Lookup the IP address from the hostname
  std::string ip;
  if (hostname != "") {
    ip = hostname_to_ip(hostname);
    saddr.sin_addr.s_addr = inet_addr(ip.c_str());
  } else {
    saddr.sin_addr.s_addr = INADDR_ANY;
  }

  if (bind(fd, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
    std::cerr << "Error binding to the UDP receive socket: " << port << std::endl;
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

  uint16_t max_queue_size;
  bool csv;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("max_queue_size,q", po::value<uint16_t>(&max_queue_size)->default_value(30),
     "the number of blocks to allow in the queue before dropping")
    ("csv,c", po::bool_switch(&csv), "output CSV log messages")
    ;

  std::string device;
  std::string mode;
  std::string conf_file;
  po::options_description pos("Positional");
  pos.add_options()
    ("device", po::value<std::string>(&device), "the wifi device to use")
    ("mode", po::value<std::string>(&mode),
     "the mode as specified in the configuration file (air/ground)")
    ("conf_file", po::value<std::string>(&conf_file),
     "the path to the configuration file used for configuring ports")
    ;
  po::positional_options_description p;
  p.add("device", 1);
  p.add("mode", 1);
  p.add("conf_file", 1);

  po::options_description all_options("Allowed options");
  all_options.add(desc).add(pos);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
	    options(all_options).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("conf_file")) {
    std::cout << "Usage: options_description [options] <device> <mode> <configuration file>\n";
    std::cout << desc;
    return EXIT_SUCCESS;
  }

  // Parse the configuration file.
  pt::ptree conf;
  try {
    pt::read_ini(conf_file, conf);
  } catch(...) {
    std::cerr << "Error reading the configuration file: " << conf_file << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<std::string> ifnames;
  get_wifi_devices(ifnames);

  // Create the message queues.
  SharedQueue<std::shared_ptr<monitor_message_t> > inqueue;   // Wifi to UDP
  SharedQueue<std::shared_ptr<Message> > outqueue;  // UDP to Wifi

  // Create the uplink UDP interfaces as specified in the configuration file.
  std::vector<std::shared_ptr<std::thread> > thrs;
  BOOST_FOREACH(const auto &v, conf) {

    // Only process uplink configuration entries.
    std::string direction = v.second.get<std::string>("direction", "");
    if (((direction == "down") && (mode == "air")) ||
	((direction == "up") && (mode == "ground"))) {

      // Get the name.
      std::string name = v.second.get<std::string>("name", "");

      // Get the UDP port number (required).
      uint16_t inport = v.second.get<uint16_t>("inport", 0);
      if (inport == 0) {
	std::cerr << "No inport specified for " << name << std::endl;
	return EXIT_FAILURE;
      }

      // Get the remote hostname/ip (optional)
      std::string hostname = v.second.get<std::string>("inhost", "127.0.0.1");

      // Get the port number (required).
      uint8_t port = v.second.get<uint16_t>("port", 0);
      if (port == 0) {
	std::cerr << "No port specified for " << name << std::endl;
	return EXIT_FAILURE;
      }

      // Get the link type
      std::string type = v.second.get<std::string>("type", "data");

      // Get the priority (optional).
      uint8_t priority = v.second.get<uint8_t>("priority", 100);

      // Get the FEC stats (optional).
      uint16_t blocksize = v.second.get<uint16_t>("blocksize", 1500);
      uint8_t nblocks = v.second.get<uint8_t>("blocks", 0);
      uint8_t nfec_blocks = v.second.get<uint8_t>("fec", 0);

      // Get the Tx parameters (optional).
      WifiOptions opts;
      opts.data_rate = v.second.get<uint8_t>("datarate", 18);
      opts.mcs = v.second.get<uint8_t>("mcs", 0) ? true : false;
      opts.stbc = v.second.get<uint8_t>("stbc", 0) ? true : false;
      opts.ldpc = v.second.get<uint8_t>("ldpc", 0) ? true : false;

      // Create the FEC encoder if requested.
      std::shared_ptr<FECEncoder> enc;
      LinkType link_type = DATA_LINK;
      if ((type == "data") && (nblocks > 0) && (nfec_blocks > 0) && (blocksize > 0)) {
	enc.reset(new FECEncoder(nblocks, nfec_blocks, blocksize));
	link_type = DATA_LINK;
      } else if (type == "short") {
	link_type = SHORT_DATA_LINK;
      } else if (type == "rts") {
	link_type = RTS_DATA_LINK;
      }

      // Try to open the UDP socket.
      int udp_sock = open_udp_socket_for_rx(inport, hostname);
      if (udp_sock < 0) {
	std::cerr << "Error opening the UDP socket for " << name << "  ("
		  << hostname << ":" << port << std::endl;
	return EXIT_FAILURE;
      }

      // Create the receive thread for this socket
      auto uth = [udp_sock, port, enc, opts, priority, blocksize, &outqueue]() {
		   while (1) {
		     std::shared_ptr<Message> msg(new Message(blocksize, port, priority, opts,
							      enc));
		     size_t count = recv(udp_sock, msg->msg.data(), blocksize, 0);
		     if (count > 0) {
		       msg->msg.resize(count);
		       outqueue.push(msg);
		     }
		   }
		 };
      thrs.push_back(std::shared_ptr<std::thread>(new std::thread(uth)));
    }    
  }

  // Open the raw transmit socket
  RawSendSocket raw_send_sock((mode == "ground"));
  if (!raw_send_sock.add_device(device)) {
    std::cerr << "Error opeing the raw socket for transmiting.\n";
    std::cerr << "  " << raw_send_sock.error_msg() << std::endl;
    return EXIT_FAILURE;
  }

  // Create a thread to send raw socket packets.
  auto send_th = [&outqueue, &raw_send_sock, csv, max_queue_size]() {
    double start = cur_time();
    double cur = 0;
    double enc_time = 0;
    double send_time = 0;
    double loop_time = 0;
    size_t count = 0;
    size_t pkts = 0;
    size_t nblocks = 0;
    size_t max_pkt = 0;
    size_t dropped_blocks = 0;

    // Send message out of the send queue
    while(1) {

      // Pull the next packet off the queue
      std::shared_ptr<Message> msg = outqueue.pop();
      double loop_start = cur_time();

      // FEC encode the packet if requested.
      if (msg->enc) {
	auto dec = msg->enc;
	// Get a FEC encoder block
	std::shared_ptr<FECBlock> block = dec->get_next_block(msg->msg.size());
	// Copy the data into the block
	std::copy(msg->msg.data(), msg->msg.data() + msg->msg.size(), block->data());
	// Pass it off to the FEC encoder.
	dec->add_block(block);
	enc_time += (cur_time() - loop_start);
	max_pkt = std::max(static_cast<size_t>(msg->msg.size()), max_pkt);
	// Transmit any packets that are finished in the encoder.
	for (block = dec->get_block(); block; block = dec->get_block()) {
	  // If the link slower than the data rate we need to drop some packets.
	  if (block->is_fec_block() & (dec->n_output_blocks() > max_queue_size)) {
	    ++dropped_blocks;
	    continue;
	  }
	  raw_send_sock.send(block->pkt_data(), block->pkt_length(), msg->port, msg->opts.link_type,
			     msg->opts.data_rate, msg->opts.mcs, msg->opts.stbc, msg->opts.ldpc);
	  count += block->pkt_length();
	  ++nblocks;
	}
	send_time += cur_time() - loop_start;
      } else {
	double send_start = cur_time();
	raw_send_sock.send(msg->msg, msg->port, msg->opts.link_type);
	send_time += (cur_time() - send_start);
	count += msg->msg.size();
	max_pkt = std::max(msg->msg.size(), max_pkt);
	++nblocks;
      }
      double cur = cur_time();
      double dur = cur - start;
      loop_time += (cur - loop_start);
      if (dur > 2.0) {
	if (csv) {
	  std::cout << pkts << "," << count << "," << dropped_blocks << "," << max_pkt << std::endl;
	} else {
	  std::cerr << "Packets/sec: " << int(nblocks / dur)
		    << " Mbps: " << 8e-6 * count / dur
		    << " Dropped: " << dropped_blocks
		    << " Max packet: " << max_pkt
		    << " Encode ms: " << 1e+3 * enc_time
		    << " Send ms: " << 1e+3 * send_time
		    << " Loop time ms: " << 1e3 * loop_time << std::endl;
	}
	start = cur;
	count = pkts = nblocks =  max_pkt = enc_time = send_time = loop_time = dropped_blocks = 0;
      }
    }
  };
  std::thread send_thr(send_th);

  // Open the raw receive socket
  RawReceiveSocket raw_recv_sock((mode == "ground"));
  if (!raw_recv_sock.add_device(device)) {
    std::cerr << "Error opeing the raw socket for transmiting.\n";
    std::cerr << "  " << raw_recv_sock.error_msg() << std::endl;
    return EXIT_FAILURE;
  }

  // Create the raw socket receive thread
  bool done = false;
  auto recv = [&raw_recv_sock, &inqueue, &done]() {
    double prev_time = cur_time();
    StatsAccumulator<int8_t> rssi_stats;
    RawReceiveStats prev_stats;
    while(!done) {
      std::shared_ptr<monitor_message_t> msg(new monitor_message_t);
      if (raw_recv_sock.receive(*msg)) {
	inqueue.push(msg);
	rssi_stats.add(msg->rssi);

	double dur = (cur_time() - prev_time);
	if (dur > 2.0) {
	  prev_time = cur_time();
	  const RawReceiveStats &stats = raw_recv_sock.stats();
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
	std::cerr << "Error receiving packet.\n" << raw_recv_sock.error_msg() << std::endl;
      }
    }
  };
  std::thread recv_thread(recv);

  // Open the UDP send socket
  int send_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (send_sock < 0) {
    std::cerr << "Error opening the UDP send socket.\n";
    return EXIT_FAILURE;
  }
  int trueflag = 1;
  if (setsockopt(send_sock, SOL_SOCKET, SO_BROADCAST, &trueflag, sizeof(trueflag)) < 0) {
    std::cerr << "Error setting the UDP send socket to broadcast.\n";
    return EXIT_FAILURE;
  }

  // Create the downlink UDP interfaces as specified in the configuration file.
  std::vector<std::shared_ptr<UDPDestination> > udp_out(16);
  BOOST_FOREACH(const auto &v, conf) {

    // Only process uplink configuration entries.
    std::string direction = v.second.get<std::string>("direction", "");
    if (((direction == "up") && (mode == "air")) ||
	((direction == "down") && (mode == "ground"))) {

      // Get the name.
      std::string name = v.second.get<std::string>("name", "");

      // Get the UDP port number (required).
      uint16_t outport = v.second.get<uint16_t>("outport", 0);
      if (outport == 0) {
	std::cerr << "No outport specified for " << name << std::endl;
	return EXIT_FAILURE;
      }

      // Get the remote hostname/ip (optional)
      std::string hostname = v.second.get<std::string>("outhost", "127.0.0.1");

      // Get the port number (required).
      uint8_t port = v.second.get<uint16_t>("port", 0);
      if (port == 0) {
	std::cerr << "No port specified for " << name << std::endl;
	return EXIT_FAILURE;
      }
      if (port > 15) {
	std::cerr << "Invalid port specified for " << name << "  (" << port << ")" << std::endl;
	return EXIT_FAILURE;
      }

      // Get the link type
      std::string type = v.second.get<std::string>("type", "data");

      // Get the FEC stats (optional).
      uint16_t blocksize = v.second.get<uint16_t>("blocksize", 1500);
      uint8_t nblocks = v.second.get<uint8_t>("blocks", 0);
      uint8_t nfec_blocks = v.second.get<uint8_t>("fec", 0);

      // Create the FEC encoder if requested.
      std::shared_ptr<FECDecoder> enc;
      LinkType link_type = DATA_LINK;
      if ((type == "data") && (nblocks > 0) && (nfec_blocks > 0) && (blocksize > 0)) {
	enc.reset(new FECDecoder());
	link_type = DATA_LINK;
      } else if (type == "short") {
	link_type = SHORT_DATA_LINK;
      } else if (type == "rts") {
	link_type = RTS_DATA_LINK;
      }

      udp_out[port].reset(new UDPDestination(outport, hostname, enc));
    }
  }

  // Retrieve messages from the raw data socket.
  double prev_time = cur_time();
  FECDecoderStats prev_stats;
  while (!done) {

    // Ralink and Atheros both always supply the FCS to userspace, no need to check
    //if (prd.m_nRadiotapFlags & IEEE80211_RADIOTAP_F_FCS)
    //bytes -= 4;

    //rx_status->adapter[adapter_no].received_packet_cnt++;
    //	rx_status->adapter[adapter_no].last_update = dbm_ts_now[adapter_no];
    //	fprintf(stderr,"lu[%d]: %lld\n",adapter_no,rx_status->adapter[adapter_no].last_update);
    //	rx_status->adapter[adapter_no].last_update = current_timestamp();

    // Pull the next block off the message queue.
    std::shared_ptr<monitor_message_t> buf = inqueue.pop();

    // Lookup the destination class.
    if (!udp_out[buf->port]) {
      std::cerr << "Error finding the output destination for port " << buf->port << std::endl;
      continue;
    }

    // Is the packet FEC encoded?
    if (udp_out[buf->port]->fec) {
      std::shared_ptr<FECDecoder> fec = udp_out[buf->port]->fec;

      // Add this block to the FEC decoder.
      fec->add_block(buf->data.data(), buf->data.size());

      // Output any packets that are finished in the decoder.
      for (std::shared_ptr<FECBlock> block = fec->get_block(); block; block = fec->get_block()) {
	if (block->data_length() > 0) {
	  sendto(send_sock, block->data(), block->data_length(), 0,
		 (struct sockaddr *)&(udp_out[buf->port]->s), sizeof(struct sockaddr_in));
	}
      }

    } else {

      // Just relay the packet if we're not FEC decoding.
      sendto(send_sock, buf->data.data(), buf->data.size(), 0,
	     (struct sockaddr *)&(udp_out[buf->port]->s), sizeof(struct sockaddr_in));
    }
  }

  for(auto th : thrs) {
    th->join();
  }
  send_thr.join();

  return EXIT_SUCCESS;
}
