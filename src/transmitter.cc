
#include <iostream>

#include <transmitter.hh>

Transmitter::Transmitter() : m_joystick(0) {
  // Try to connect to the transmitter
  connect();
}

bool Transmitter::connect() {

  // Connect to the joystick if it's disconnected
  if (!m_joystick) {

    // Search for the Taranis Tx
    for (uint8_t i = 0; i < SDL_NumJoysticks(); ++i) {
      if (m_joystick = SDL_JoystickOpen(i)) {
	if (std::string("FrSky") == std::string(SDL_JoystickName(m_joystick)).substr(0, 5)) {
	  break;
	}
	SDL_JoystickClose(m_joystick);
	m_joystick = 0;
      }
    }
  }

  return connected();
}

uint16_t Transmitter::channel(uint8_t idx) {
  // Try to connect if we're not yet connected.
  connect();

  if (connected()) {
    return static_cast<uint16_t>((static_cast<float>(SDL_JoystickGetAxis(m_joystick, idx)) /
				  65536.0 + 0.5) * 1000.0 + 1000.0);
  } else {
    return 0;
  }
}

bool Transmitter::connected() const {
  return m_joystick;
}
