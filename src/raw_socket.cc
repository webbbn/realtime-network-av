
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
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

#include <iostream>

#include <raw_socket.hh>

#include <pcap-bpf.h>
#include <radiotap.h>

typedef struct {
  pcap_t *ppcap;
  int selectable_fd;
  int n80211HeaderLength;
} monitor_interface_t;

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
  0x13, 0x22, 0x33, 0x44, 0x55, 0x66, // receiver mac address (last byte is port + link type)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // transmitter mac address (1-4 seq num 5-6 udp port)
  0x00, 0x00 // IEEE802.11 seqnum, (will be overwritten later by Atheros firmware/wifi chip)
};

static uint8_t u8aIeeeHeader_rts[] = {
  0xb4, 0x01, 0x00, 0x00, // frame control field (2 bytes), duration (2 bytes)
  0xff, //  port = 1st byte of IEEE802.11 RA (mac) must be something odd
  // (wifi hardware determines broadcast/multicast through odd/even check)
};

/******************************************************************************
 * RawSendSocket
 *****************************************************************************/

RawSendSocket::RawSendSocket(bool ground, uint32_t send_buffer_size, uint32_t max_packet) :
  m_ground(ground), m_max_packet(max_packet), m_seq_num(0) {

  // Create the send buffer with the appropriate headers.
  m_hdr_len = sizeof(radiotap_header) + sizeof(ieee_header_data);
  m_send_buf.resize(m_hdr_len + max_packet);
  memcpy(m_send_buf.data(), radiotap_header, sizeof(radiotap_header));
  memcpy(m_send_buf.data() + sizeof(radiotap_header), ieee_header_data, sizeof(ieee_header_data));
}

bool RawSendSocket::add_device(const std::string &device) {

  m_sock = socket(AF_PACKET, SOCK_RAW, 0);
  if (m_sock == -1) {
    m_error_msg = "Socket open failed.";
    return false;
  }

  struct sockaddr_ll ll_addr;
  ll_addr.sll_family = AF_PACKET;
  ll_addr.sll_protocol = 0;
  ll_addr.sll_halen = ETH_ALEN;

  struct ifreq ifr;
  strncpy(ifr.ifr_name, device.c_str(), IFNAMSIZ);

  if (ioctl(m_sock, SIOCGIFINDEX, &ifr) < 0) {
    m_error_msg = "Error: ioctl(SIOCGIFINDEX) failed.";
    close(m_sock);
    m_sock = -1;
    return false;
  }
  ll_addr.sll_ifindex = ifr.ifr_ifindex;

  if (ioctl(m_sock, SIOCGIFHWADDR, &ifr) < 0) {
    m_error_msg = "Error: ioctl(SIOCGIFHWADDR) failed.";
    close(m_sock);
    m_sock = -1;
    return false;
  }
  memcpy(ll_addr.sll_addr, ifr.ifr_hwaddr.sa_data, ETH_ALEN);

  if (bind(m_sock, (struct sockaddr *)&ll_addr, sizeof(ll_addr)) == -1) {
    m_error_msg = "Error: bind failed.";
    close(m_sock);
    m_sock = -1;
    return false;
  }

  if (m_sock == -1) {
    m_error_msg = "Error: Cannot open socket: Must be root with an 802.11 card with RFMON enabled";
    return false;
  }

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 8000;
  if (setsockopt(m_sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout)) < 0) {
    m_error_msg = "setsockopt SO_SNDTIMEO";
    return false;
  }

  if (setsockopt(m_sock, SOL_SOCKET, SO_SNDBUF, &m_buffer_size, sizeof(m_buffer_size)) < 0) {
    m_error_msg = "setsockopt SO_SNDBUF";
    return false;
  }

  return true;
}

uint8_t *RawSendSocket::send_buffer() {
  return m_send_buf.data() + m_hdr_len;
}

bool RawSendSocket::send(size_t msglen, uint8_t port, LinkType type) {
  // Set the sequence number
  ++m_seq_num;
  uint32_t *seq_num_ptr =
    reinterpret_cast<uint32_t*>(m_send_buf.data() + sizeof(radiotap_header) + 16);
  *seq_num_ptr = m_seq_num;

  // Set the port in the header
  m_send_buf[sizeof(radiotap_header) + 4] = (((port & 0xf) << 4) | (m_ground ? 0xd : 0x5));

  // Send the packet
  return (::send(m_sock, m_send_buf.data(), msglen + m_hdr_len, 0) >= 0);
}

bool RawSendSocket::send(const uint8_t *msg, size_t msglen, uint8_t port, LinkType type) {
  memcpy(send_buffer(), msg, msglen);
  return send(msglen, port, type);
}

/******************************************************************************
 * RawReceiveSocket
 *****************************************************************************/

RawReceiveSocket::RawReceiveSocket(bool ground, uint32_t max_packet) :
  m_ground(ground), m_max_packet(max_packet) {
}

bool RawReceiveSocket::add_device(const std::string &device) {

  // open the interface in pcap
  char errbuf[PCAP_ERRBUF_SIZE];
  errbuf[0] = '\0';
  m_ppcap = pcap_open_live(device.c_str(), 2350, 0, -1, errbuf);
  if (m_ppcap == NULL) {
    m_error_msg = "Unable to open " + device + ": " + std::string(errbuf);
    return false;
  }

/*
  if(pcap_setnonblock(interface->ppcap, 1, errbuf) < 0) {
    std::cerr << "Error setting " << device << " to nonblocking mode: " << errbuf << std::endl;
    return false;
  }
*/

  if(pcap_setdirection(m_ppcap, PCAP_D_IN) < 0) {
    m_error_msg = "Error setting " + device + " direction\n";
    return false;
  }

  int nLinkEncap = pcap_datalink(m_ppcap);
  if (nLinkEncap != DLT_IEEE802_11_RADIO) {
    m_error_msg = "ERROR: unknown encapsulation on " + device +
      "! check if monitor mode is supported and enabled";
    return false;
  }

  // Match the first 4 bytes of the destination address.
  struct bpf_program bpfprogram;
  const char *filter_gnd = "(ether[0x00:2] == 0x0801 || ether[0x00:2] == 0x0802 || ether[0x00:4] == 0xb4010000) && ((ether[0x04:1] & 0x0f) == 0x05)";
  const char *filter_air = "(ether[0x00:2] == 0x0801 || ether[0x00:2] == 0x0802 || ether[0x00:4] == 0xb4010000) && ((ether[0x04:1] & 0x0f) == 0x0d)";
  const char *filter = (m_ground ? filter_gnd : filter_air);
  if (pcap_compile(m_ppcap, &bpfprogram, filter, 1, 0) == -1) {
    m_error_msg = "Error compiling bpf program: " + std::string(filter);
    return false;
  }

  // Configure the filter.
  if (pcap_setfilter(m_ppcap, &bpfprogram) == -1) {
    m_error_msg = "Error configuring the bpf program: " + std::string(filter);
    return false;
  }
  pcap_freecode(&bpfprogram);

  m_selectable_fd = pcap_get_selectable_fd(m_ppcap);

  return true;
}

bool RawReceiveSocket::receive(monitor_message_t &msg) {
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
    int retval = pcap_next_ex(m_ppcap, &pcap_packet_header, &pcap_packet_data);
    if (retval < 0) {
      m_error_msg = "Error receiving from the raw data socket.\n  " +
	std::string(pcap_geterr(m_ppcap));
      continue;
    } else if(retval == 0) {
      // Timeout, just continue;
      continue;
    }

    break;
  }

  // fetch radiotap header length from radiotap header (seems to be 36 for Atheros and 18 for Ralink)
  uint16_t rt_header_len = (pcap_packet_data[3] << 8) + pcap_packet_data[2];

  // check for packet type and set headerlen accordingly
  pcap_packet_data += rt_header_len;
  switch (pcap_packet_data[1]) {
  case 0x01: // data short, rts
    m_n80211HeaderLength = 0x05;
    break;
  case 0x02: // data
    m_n80211HeaderLength = 0x18;
    msg.seq_num = *reinterpret_cast<const uint32_t*>(pcap_packet_data + 16);
    break;
  default:
    break;
  }
  msg.port = (pcap_packet_data[4] >> 4);
  pcap_packet_data -= rt_header_len;

  if (pcap_packet_header->len < static_cast<uint32_t>(rt_header_len + m_n80211HeaderLength)) {
    m_error_msg = "rx ERROR: ppcapheaderlen < u16headerlen + n80211headerlen";
    return false;
  }

  struct ieee80211_radiotap_iterator rti;
  if (ieee80211_radiotap_iterator_init(&rti,(struct ieee80211_radiotap_header *)pcap_packet_data,
				       pcap_packet_header->len) < 0) {
    m_error_msg = "rx ERROR: radiotap_iterator_init < 0";
    return false;
  }

  int n;
  while ((n = ieee80211_radiotap_iterator_next(&rti)) == 0) {
    switch (rti.this_arg_index) {
    case IEEE80211_RADIOTAP_RATE:
      msg.rate = (*rti.this_arg);
      break;
    case IEEE80211_RADIOTAP_CHANNEL:
      msg.channel = *((uint16_t *)rti.this_arg);
      msg.channel_flag = *((uint16_t *)(rti.this_arg + 2));
      break;
    case IEEE80211_RADIOTAP_ANTENNA:
      msg.antenna = (*rti.this_arg) + 1;
      break;
    case IEEE80211_RADIOTAP_FLAGS:
      msg.radiotap_flags = *rti.this_arg;
      break;
    case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
      msg.rssi = (int8_t)(*rti.this_arg);
      break;
    }
  }

  // Copy the data into the message buffer.
  const uint32_t crc_len = 4;
  uint32_t header_len = rt_header_len + m_n80211HeaderLength;
  uint32_t packet_len = pcap_packet_header->len - header_len - crc_len;
  msg.data.resize(packet_len);
  std::copy(pcap_packet_data + header_len, pcap_packet_data + header_len + packet_len,
	    msg.data.begin());

  // Validate the sequence number.
  if (m_stats.packets == 0) {
    m_stats.prev_good_seq_num = msg.seq_num;
  } else if (msg.seq_num == (m_stats.seq_num + 1)) {
    // This is what we want
    ++m_stats.good_packets;
    m_stats.cur_error_count = 0;
    m_stats.prev_good_seq_num = msg.seq_num;
  } else if (msg.seq_num > m_stats.prev_good_seq_num) {
    uint32_t diff = msg.seq_num - m_stats.prev_good_seq_num;
    if (diff < 5) {
      // This likely means we dropped a few packets.
      m_stats.dropped_packets += (diff - 1);
      m_stats.cur_error_count = 0;
      m_stats.prev_good_seq_num = msg.seq_num;
    } else {
      // Something's wrong. Just drop this packet until we get enough to force a resync.
      ++m_stats.error_packets;
      ++m_stats.cur_error_count;
      if (m_stats.cur_error_count > 5) {
	// Resync
	m_stats.cur_error_count = 0;
	m_stats.prev_good_seq_num = msg.seq_num;
	++m_stats.resets;
      }
    }
  } else {
    // Something's wrong. Just drop this packet until we get enough to force a resync.
    ++m_stats.error_packets;
    ++m_stats.cur_error_count;
    if (m_stats.cur_error_count > 5) {
      // Resync
      m_stats.cur_error_count = 0;
      m_stats.prev_good_seq_num = msg.seq_num;
      ++m_stats.resets;
    }
  }
  m_stats.seq_num = msg.seq_num;
  ++m_stats.packets;
  m_stats.bytes += packet_len;


  return true;
}
