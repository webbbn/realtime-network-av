
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

#include <boost/foreach.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <shared_queue.hh>
#include <raw_socket.hh>
#include <fec.hh>

namespace po=boost::program_options;
namespace pt=boost::property_tree;

static const uint32_t max_packet = 65000;

struct Message {
  Message(size_t max_packet, uint16_t p, LinkType lt, std::shared_ptr<FECEncoder> e) :
    msg(max_packet), port(p), link_type(lt), enc(e) { }
  std::vector<uint8_t> msg;
  uint16_t port;
  LinkType link_type;
  std::shared_ptr<FECEncoder> enc;
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
  uint16_t max_queue_size;
  bool csv;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("blocks_size,b", po::value<uint16_t>(&block_size)->default_value(1024),
     "the size of the FEC blocks")
    ("nblocks,n", po::value<uint16_t>(&nblocks)->default_value(8),
     "the number of data blockes used in encoding")
    ("nfec_blocks,k", po::value<uint16_t>(&nfec_blocks)->default_value(4),
     "the number of FEC blockes used in encoding")
    ("max_queue_size,q", po::value<uint16_t>(&max_queue_size)->default_value(30),
     "the number of blocks to allow in the queue before dropping")
    ("csv,c", po::bool_switch(&csv), "output CSV log messages")
    ;

  std::string device;
  uint16_t port;
  std::string conf_file;
  po::options_description pos("Positional");
  pos.add_options()
    ("device", po::value<std::string>(&device), "the wifi device to use")
    ("port", po::value<uint16_t>(&port), "the receivers port number")
    ("conf_file", po::value<std::string>(&conf_file),
     "the path to the configuration file used for configuring ports")
    ;
  po::positional_options_description p;
  p.add("device", 1);
  p.add("port", 1);
  p.add("conf_file", 1);

  po::options_description all_options("Allowed options");
  all_options.add(desc).add(pos);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
	    options(all_options).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("conf_file")) {
    std::cout << "Usage: options_description [options] <device> <port> <configuration file>\n";
    std::cout << desc;
    return EXIT_SUCCESS;
  }

  // Create the message queue.
  SharedQueue<std::shared_ptr<Message> > queue;

  // Parse the configuration file.
  pt::ptree conf;
  try {
    pt::read_json(conf_file, conf);
  } catch(...) {
    std::cerr << "Error reading the configuration file: " << conf_file << std::endl;
    return EXIT_FAILURE;
  }

  // Create the interfaces as specified in the configuration file.
  std::vector<std::shared_ptr<std::thread> > thrs;
  BOOST_FOREACH(const auto &v, conf.get_child("wifi_bridge_ground.ports")) {
    std::string name = v.second.get<std::string>("name", "");

    // Get the port number (required).
    uint16_t port = v.second.get<uint16_t>("port", 0);
    if (port == 0) {
      std::cerr << "No port specified for " << name << std::endl;
      return EXIT_FAILURE;
    }

    // Get the remote hostname (optional)
    std::string hostname = v.second.get<std::string>("host", "");

    // Get the link type
    std::string type = v.second.get<std::string>("type", "data");

    // Try to open the UDP socket.
    int udp_sock = open_udp_socket_for_rx(port, hostname);
    if (udp_sock < 0) {
      std::cerr << "Error opening the UDP socket for " << name << std::endl;
      return EXIT_FAILURE;
    }

    // Create the FEC encoder if requested.
    std::shared_ptr<FECEncoder> enc;
    LinkType link_type = DATA_LINK;
    if (type == "data") {
      link_type = DATA_LINK;
    } else if (type == "fec") {
      link_type = FEC_LINK;
      enc.reset(new FECEncoder(nblocks, nfec_blocks, block_size, false));
    } else if (type == "wfb") {
      link_type = WFB_LINK;
      enc.reset(new FECEncoder(nblocks, nfec_blocks, block_size, true));
    } else if (type == "short") {
      link_type = SHORT_DATA_LINK;
    } else if (type == "rts") {
      link_type = RTS_DATA_LINK;
    }

    // Create the receive thread for this socket
    auto uth = [udp_sock, port, enc, link_type, &queue]() {
		 while (1) {
		   std::shared_ptr<Message> msg(new Message(max_packet, port, link_type, enc));
		   size_t count = recv(udp_sock, msg->msg.data(), max_packet, 0);
		   if (count > 0) {
		     msg->msg.resize(count);
		     queue.push(msg);
		   }
		 }
	       };
    thrs.push_back(std::shared_ptr<std::thread>(new std::thread(uth)));
  }

  // Open the raw socket
  RawSendSocket raw_sock(port);
  if (!raw_sock.add_device(device)) {
    std::cerr << "Error opeing the raw socket for transmiting.\n";
    std::cerr << "  " << raw_sock.error_msg() << std::endl;
    return EXIT_FAILURE;
  }

  // Create a thread to send packets.
  auto send_th = [&queue, &raw_sock, csv, max_queue_size]() {
    double start = cur_time();
    double cur = 0;
    double enc_time = 0;
    double send_time = 0;
    double loop_time = 0;
    size_t count = 0;
    size_t pkts = 0;
    size_t blocks = 0;
    size_t max_pkt = 0;
    size_t dropped_blocks = 0;

    // Send message out of the send queue
    while(1) {

      // Pull the next packet off the queue
      std::shared_ptr<Message> msg = queue.pop();
      while(queue.size() > max_queue_size) {
	msg = queue.pop();
	++dropped_blocks;
      }
      ++pkts;
      double loop_start = cur_time();

      // FEC encode the packet if requested.
      if (msg->enc) {
	msg->enc->encode(msg->msg.data(), msg->msg.size());
	enc_time += (cur_time() - loop_start);
	max_pkt = std::max(static_cast<size_t>(msg->msg.size()), max_pkt);
	for (const uint8_t *block : msg->enc->blocks()) {
	  raw_sock.send(block, msg->enc->block_size() + 4, msg->port, msg->link_type);
	  count += msg->enc->block_size() + 4;
	  ++blocks;
	}
	send_time += cur_time() - loop_start;
      } else {
	double send_start = cur_time();
	raw_sock.send(msg->msg, msg->port, msg->link_type);
	send_time += (cur_time() - send_start);
	count += msg->msg.size();
	max_pkt = std::max(msg->msg.size(), max_pkt);
	++pkts;
	++blocks;
      }
      double cur = cur_time();
      double dur = cur - start;
      loop_time += (cur - loop_start);
      if (dur > 2.0) {
	if (csv) {
	  std::cout << pkts << "," << count << "," << dropped_blocks << "," << max_pkt << std::endl;
	} else {
	  std::cerr << "Blocks/sec: " << int(blocks / dur)
		    << " Packets/sec: " << int(pkts / dur)
		    << " Mbps: " << 8e-6 * count / dur
		    << " Dropped: " << dropped_blocks
		    << " Max packet: " << max_pkt
		    << " Encode ms: " << 1e+3 * enc_time
		    << " Send ms: " << 1e+3 * send_time
		    << " Loop time ms: " << 1e3 * loop_time << std::endl;
	}
	start = cur;
	count = pkts = blocks =  max_pkt = enc_time = send_time = loop_time = dropped_blocks = 0;
      }
    }
  };
  std::thread send_thr(send_th);

  for(auto th : thrs) {
    th->join();
  }
  send_thr.join();

  return EXIT_SUCCESS;
}
