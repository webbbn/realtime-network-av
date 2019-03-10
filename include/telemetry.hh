#ifndef RTNAV_TELEMETRY_HH
#define RTNAV_TELEMETRY_HH

#include <string>
#include <map>

#include <boost/asio.hpp>

struct Telemetry {

  Telemetry(boost::asio::io_context &io_context);

  bool get_value(const std::string &name, float &value);

private:
  typedef std::map<std::string, float> NVMap;

  void set_value(const std::string &name, float value);

  void reader_thread();

  boost::asio::ip::udp::socket m_sock;
  NVMap m_values;
};

#endif /* RTNAV_TELEMETRY_HH */
