
#include <iostream>
#include <thread>

#include <mavlink.h>

#include "telemetry.hh"

Telemetry::Telemetry(io_context &io_context) :
  m_sock(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), 14550)) {
  m_sock.set_option(boost::asio::socket_base::broadcast(true));
  std::thread([this]() { this->reader_thread(); }).detach();
  //mavlink_message_t msg;
  //mavlink_msg_request_data_stream_pack(99, 99, &msg, 1, 0, MAV_DATA_STREAM_EXTENDED_STATUS, 5, 1)
}

bool Telemetry::get_value(const std::string &name, float &value) {
  NVMap::const_iterator mi = m_values.find(name);
  if (mi == m_values.end()) {
    return false;
  }
  value = mi->second;
  return true;
}

void Telemetry::set_value(const std::string &name, float value) {
  m_values[name] = value;
}

void Telemetry::reader_thread() {
  int max_length = 1024;
  mavlink_message_t msg;
  mavlink_status_t status;
  char data[max_length];
  while(1) {
    boost::asio::ip::udp::endpoint sender_endpoint;
    size_t length = m_sock.receive_from(boost::asio::buffer(data, max_length), sender_endpoint);

    for (size_t i = 0; i < length; ++i) {
      //temp = data[i];
      //printf("%02x ", (unsigned char)temp);
      if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
	switch (msg.msgid) {
	case MAVLINK_MSG_ID_POWER_STATUS:
	  break;
	case MAVLINK_MSG_ID_SYS_STATUS:
	  mavlink_sys_status_t sys_status;
	  mavlink_msg_sys_status_decode(&msg, &sys_status);
	  set_value("voltage_battery", sys_status.voltage_battery);
	  set_value("current_battery", sys_status.current_battery);
	  set_value("battery_remaining", sys_status.battery_remaining);
	  break;
	case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
	  mavlink_nav_controller_output_t nav;
	  mavlink_msg_nav_controller_output_decode(&msg, &nav);
	  break;
	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
	  mavlink_global_position_int_t pos;
	  mavlink_msg_global_position_int_decode(&msg, &pos);
	  set_value("latitude", static_cast<float>(pos.lat) * 1e-7);
	  set_value("longitude", static_cast<float>(pos.lon) * 1e-7);
	  set_value("altitude", static_cast<float>(pos.alt) / 1000.0);
	  set_value("relative_altitude", static_cast<float>(pos.relative_alt) / 1000.0);
	  set_value("speed", sqrt(pos.vx * pos.vx + pos.vy * pos.vy + pos.vz * pos.vz) / 100.0);
	  set_value("heading", static_cast<float>(pos.hdg) / 100.0);
	  break;
	case MAVLINK_MSG_ID_ATTITUDE:
	  mavlink_attitude_t att;
	  mavlink_msg_attitude_decode(&msg, &att);
	  break;
	case MAVLINK_MSG_ID_STATUSTEXT:
	  mavlink_statustext_t status;
	  mavlink_msg_statustext_decode(&msg, &status);
	  break;
	case MAVLINK_MSG_ID_MISSION_CURRENT:
	  //std::cerr << "Mission current " << std::endl;
	  break;
	case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
	  //std::cerr << "Servo raw " << std::endl;
	  break;
	case MAVLINK_MSG_ID_RC_CHANNELS:
	  //std::cerr << "RC Channels " << std::endl;
	  break;
	case MAVLINK_MSG_ID_PARAM_VALUE:
	  //std::cerr << "Param value " << std::endl;
	  break;
	case MAVLINK_MSG_ID_VIBRATION:
	  //std::cerr << "Vibration " << std::endl;
	  break;
	case MAVLINK_MSG_ID_HEARTBEAT:
	  mavlink_heartbeat_t hb;
	  mavlink_msg_heartbeat_decode(&msg, &hb);
	  set_value("armed", (hb.base_mode & 0x80) ? 1.0 : 0.0);
	  set_value("mode", static_cast<float>(hb.custom_mode));
	  break;
	case MAVLINK_MSG_ID_VFR_HUD:
	  //std::cerr << "VFR HUD " << std::endl;
	  break;
	case MAVLINK_MSG_ID_RAW_IMU:
	  //std::cerr << "Raw IMU " << std::endl;
	  break;
	case MAVLINK_MSG_ID_SCALED_PRESSURE:
	  //std::cerr << "Scaled Pressure " << std::endl;
	  break;
	case MAVLINK_MSG_ID_GPS_RAW_INT:
	  //std::cerr << "GSP Raw " << std::endl;
	  break;
	case MAVLINK_MSG_ID_SYSTEM_TIME:
	  //std::cerr << "System Time " << std::endl;
	  break;
	case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
	  //std::cerr << "Local position " << std::endl;
	  break;
	case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
	  //std::cerr << "Autopilot version " << std::endl;
	  break;
	case MAVLINK_MSG_ID_COMMAND_ACK:
	  //std::cerr << "Command ACK " << std::endl;
	  break;
	default:
	  std::cerr << "Received packet: SYS: " << msg.sysid
		    << ", COMP: " << int(msg.compid)
		    << ", LEN: " << int(msg.len)
		    << ", MSG ID: " << msg.msgid << std::endl;
	  break;
	}
      }
    }
  }
}
