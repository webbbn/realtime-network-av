#ifndef RTNAV_SDL_RENDER_WINDOW_HH
#define RTNAV_SDL_RENDER_WINDOW_HH

#include <SDL.h>

#include "sdl_osd.hh"
#include "telemetry.hh"

class SDLRenderWindow {
public:

  SDLRenderWindow(std::shared_ptr<Telemetry> telem,
		  const std::string &font_file,
		  const std::string &home_dir_icon,
		  const std::string &north_arrow_icon,
		  uint16_t screen,
		  bool fullstcreen);

  ~SDLRenderWindow();

  bool good() const;

  void update(uint32_t width, uint32_t height,
	      uint8_t *y_plane, uint8_t *u_plane, uint8_t *v_plane);

private:
  std::shared_ptr<Telemetry> m_telem;
  std::string m_font_file;
  std::string m_home_dir_icon;
  std::string m_north_arrow_icon;
  std::shared_ptr<SDLOSD> m_osd;
  uint16_t m_screen;
  SDL_Window *m_win;
  SDL_Renderer *m_renderer;
  SDL_Texture *m_texture;
  uint32_t m_screen_width;
  uint32_t m_screen_height;
  bool m_fullscreen;
};

#endif /* RTNAV_SDL_RENDER_WINDOW_HH */
