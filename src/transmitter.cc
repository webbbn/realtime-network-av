
#include <iostream>

#include <transmitter.hh>

Transmitter::Transmitter() : m_joystick(0) {

  // Search for the Taranis Tx
  for (uint8_t i = 0; i < SDL_NumJoysticks(); ++i) {
    if (m_joystick = SDL_JoystickOpen(i)) {
      if (std::string("FrSky Taranis Joystick") == SDL_JoystickName(m_joystick)) {
	break;
      }
      SDL_JoystickClose(m_joystick);
      m_joystick = 0;
    }
  }

  // Resize the axes array to hold the axes values.
  for (uint8_t i = 0; i < SDL_JoystickNumAxes(m_joystick); ++i) {
    m_channels.push_back(0);
  }
}

void Transmitter::update(const SDL_Event &ev) {
  if (ev.type == SDL_JOYAXISMOTION) {
    m_channels[ev.jaxis.axis] =
      static_cast<uint16_t>((static_cast<float>(ev.jaxis.value) / 65536.0 + 0.5) * 1900.0);
  }
}

float Transmitter::channel(uint8_t idx) const {
  return m_channels[idx];
}

bool Transmitter::good() const {
  return m_joystick;
}
