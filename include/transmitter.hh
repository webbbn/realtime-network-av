#ifndef RTNAV_TRANSMITTER_HH
#define RTNAV_TRANSMITTER_HH

#include <vector>

#include <SDL.h>

class Transmitter {
public:

  Transmitter();

  bool connect();

  bool connected() const;

  uint16_t channel(uint8_t idx);

private:
  SDL_Joystick *m_joystick;
};

#endif /* RTNAV_TRANSMITTER_HH */
