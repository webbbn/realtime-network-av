#ifndef RTNAV_TRANSMITTER_HH
#define RTNAV_TRANSMITTER_HH

#include <vector>

#include <SDL.h>

class Transmitter {
public:

  Transmitter();

  bool good() const;

  void update(const SDL_Event &event);

  float channel(uint8_t idx) const;

private:
  SDL_Joystick *m_joystick;
  std::vector<uint16_t> m_channels;
};

#endif /* RTNAV_TRANSMITTER_HH */
