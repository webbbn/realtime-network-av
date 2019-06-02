#ifndef RTNAV_SDL_RENDER_WINDOW_HH
#define RTNAV_SDL_RENDER_WINDOW_HH

#include <SDL.h>

#include "sdl_osd.hh"
#include "telemetry.hh"

class SDLRenderWindow {
public:

  SDLRenderWindow(std::shared_ptr<Telemetry> telem,
		  const std::string &font_file,
		  const std::string &image_dir,
		  uint32_t wx, uint32_t wy,
		  uint8_t screen, bool windowed);

  ~SDLRenderWindow();

  bool good() const;

  void update(uint32_t video_width, uint32_t video_height,
	      uint8_t *y_plane, uint8_t *u_plane, uint8_t *v_plane);

private:
  std::shared_ptr<Telemetry> m_telem;
  std::string m_font_file;
  std::string m_image_dir;
  std::shared_ptr<SDLOSD> m_osd;
  int16_t m_screen;
  SDL_Window *m_win;
  SDL_Renderer *m_renderer;
  SDL_Texture *m_texture;
  uint32_t m_screen_width;
  uint32_t m_screen_height;
  uint32_t m_win_x;
  uint32_t m_win_y;
  bool m_windowed;
};

#endif /* RTNAV_SDL_RENDER_WINDOW_HH */
