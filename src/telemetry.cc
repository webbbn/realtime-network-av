
#include <iostream>
#include <thread>

#include <mavlink.h>

#include "telemetry.hh"

Telemetry::Telemetry(io_context &io_context, Transmitter &tx) :
  m_recv_sock(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(),
							 14550)),
  m_send_sock(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(),
							 14551)),
  m_tx(tx), m_sysid(0), m_compid(0), m_sender_valid(false), m_rec_bat_status(false) {
  m_recv_sock.set_option(boost::asio::socket_base::broadcast(true));
  m_send_sock.set_option(boost::asio::socket_base::broadcast(true));
  std::thread([this]() { this->reader_thread(); }).detach();
  std::thread([this]() { this->control_thread(); }).detach();
}

bool Telemetry::get_value(const std::string &name, float &value) const {
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
  mavlink_message_t msg;
  mavlink_status_t status;
  int max_length = 1024;
  uint8_t data[max_length];
  while(1) {
    size_t length = m_recv_sock.receive_from(boost::asio::buffer(data, max_length), m_sender_endpoint);
    m_sender_valid = true;

    for (size_t i = 0; i < length; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
	m_sysid = msg.sysid;
	m_compid = msg.compid;
	switch (msg.msgid) {
	case MAVLINK_MSG_ID_POWER_STATUS:
	  break;
	case MAVLINK_MSG_ID_SYS_STATUS:
	  mavlink_sys_status_t sys_status;
	  mavlink_msg_sys_status_decode(&msg, &sys_status);
	  //if (!m_rec_bat_status) {
	  set_value("voltage_battery", sys_status.voltage_battery / 1000.0);
	  set_value("current_battery", sys_status.current_battery);
	  set_value("battery_remaining", sys_status.battery_remaining);
	  //}
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
	  {
	    mavlink_heartbeat_t hb;
	    mavlink_msg_heartbeat_decode(&msg, &hb);
	    bool is_armed = (hb.base_mode & 0x80);
	    set_value("armed", is_armed ? 1.0 : 0.0);
	    set_value("mode", static_cast<float>(hb.custom_mode));
	  }
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
	case MAVLINK_MSG_ID_BATTERY_STATUS:
	  mavlink_battery_status_t bat;
	  mavlink_msg_battery_status_decode(&msg, &bat);
	  /*
	  set_value("voltage_battery", bat.voltages[0] / 1000.0);
	  set_value("current_battery", bat.current_battery);
	  set_value("battery_remaining", bat.battery_remaining);
	  m_rec_bat_status = true;
	  */
	  break;
	case MAVLINK_MSG_ID_HOME_POSITION:
	  mavlink_home_position_t home;
	  mavlink_msg_home_position_decode(&msg, &home);
	  set_value("home_latitude", home.latitude * 1e-7);
	  set_value("home_longitude", home.longitude * 1e-7);
	  set_value("home_altitude", home.altitude);
	  break;
	default:
	  std::cerr << "Received packet: SYS: " << int(msg.sysid)
		    << ", COMP: " << int(msg.compid)
		    << ", LEN: " << int(msg.len)
		    << ", MSG ID: " << msg.msgid << std::endl;
	  break;
	}
      }
    }
  }
}

void Telemetry::control_thread() {
  int max_length = 1024;
  uint8_t data[max_length];
  size_t counter = 0;
  bool done = false;

  // Don't send anything until we've recieved a packet.
  while (!m_sender_valid) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
    
  while (!done) {

    if (counter == 200) {
      std::cerr << "rc: " << m_tx.channel(0) << " " << m_tx.channel(1) << " "
		<< m_tx.channel(2) << " " << m_tx.channel(3) << " "
		<< m_tx.channel(4) << " " << m_tx.channel(5) << " "
		<< m_tx.channel(6) << " " << m_tx.channel(7) << std::endl;
      counter = 0;
    } else {
      ++counter;
    }

    // Send control messages every 20 ms
    {
      mavlink_rc_channels_override_t rc;
      mavlink_message_t msg_rc;
      rc.chan1_raw = m_tx.channel(1);
      rc.chan2_raw = m_tx.channel(2);
      rc.chan3_raw = m_tx.channel(0);
      rc.chan4_raw = m_tx.channel(3);
      rc.chan5_raw = m_tx.channel(4);
      rc.chan6_raw = m_tx.channel(5);
      rc.chan7_raw = m_tx.channel(6);
      rc.chan8_raw = m_tx.channel(7);
      rc.target_system = 1;
      rc.target_component = 1;
      mavlink_msg_rc_channels_override_encode(255, 0, &msg_rc, &rc);
      int len = mavlink_msg_to_send_buffer(data, &msg_rc);
      m_send_sock.send_to(boost::asio::buffer(data, len), m_sender_endpoint);
      mavlink_msg_rc_channels_override_encode(255, 1, &msg_rc, &rc);
      len = mavlink_msg_to_send_buffer(data, &msg_rc);
      m_send_sock.send_to(boost::asio::buffer(data, len), m_sender_endpoint);
    }

    // Sleep for 20 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
