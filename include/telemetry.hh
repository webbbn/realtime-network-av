#ifndef RTNAV_TELEMETRY_HH
#define RTNAV_TELEMETRY_HH

#include <string>
#include <map>

#include <boost/asio.hpp>

#include "transmitter.hh"

struct Telemetry {
#if BOOST_VERSION < 106600
  typedef boost::asio::io_service io_context;
#else
  typedef boost::asio::io_context io_context;
#endif

  Telemetry(io_context &io_context, Transmitter &tx);

  bool get_value(const std::string &name, float &value) const;

  bool armed() const;
  void armed(bool val);

private:
  typedef std::map<std::string, float> NVMap;

  void set_value(const std::string &name, float value);

  void reader_thread();
  void control_thread();

  boost::asio::ip::udp::socket m_recv_sock;
  boost::asio::ip::udp::socket m_send_sock;
  NVMap m_values;
  Transmitter &m_tx;
  uint8_t m_sysid;
  uint8_t m_compid;
  boost::asio::ip::udp::endpoint m_sender_endpoint;
  bool m_sender_valid;
};

#endif /* RTNAV_TELEMETRY_HH */
