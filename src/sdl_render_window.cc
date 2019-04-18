
#include <iostream>

#include <SDL_ttf.h>

#include "sdl_render_window.hh"

SDLRenderWindow::SDLRenderWindow(std::shared_ptr<Telemetry> telem,
				 const std::string &font_file,
				 const std::string &home_dir_icon,
				 const std::string &north_arrow_icon,
				 uint16_t screen,
				 bool fullscreen) :
  m_telem(telem), m_font_file(font_file), m_home_dir_icon(home_dir_icon),
  m_north_arrow_icon(north_arrow_icon), m_screen(screen), m_win(0), m_renderer(0), m_texture(0),
  m_fullscreen(fullscreen) {

  // Initialize SDL_ttf library
  if (TTF_Init() != 0) {
    std::cerr << "TTF_Init() Failed: " << TTF_GetError() << std::endl;
    SDL_Quit();
    exit(1);
  }

  // Get the screen size
  SDL_DisplayMode DM;
  SDL_GetCurrentDisplayMode(0, &DM);
  std::cerr << "DS: " << DM.w << "x" << DM.h << std::endl;
  // This is a hack to support windowed mode
  if (DM.w <= 1920) {
    m_screen_width = DM.w;
    m_screen_height = DM.h;
  } else {
    m_screen_width = 0;
    m_screen_height = 0;
  }
}

SDLRenderWindow::~SDLRenderWindow() {
  if (m_texture) {
    SDL_DestroyTexture(m_texture);
  }
  if (m_renderer) {
    SDL_DestroyRenderer(m_renderer);
  }
  if (m_win) {
    SDL_DestroyWindow(m_win);
  }
}

bool SDLRenderWindow::good() const {
  return m_win && m_texture && m_renderer;
}

void SDLRenderWindow::update(uint32_t width, uint32_t height, uint8_t *y_plane, uint8_t *u_plane,
			     uint8_t *v_plane) {

  // Create the playback window.
  if (!m_win) {
    if (m_fullscreen && (SDL_GetNumVideoDisplays() >= m_screen)) {
      SDL_Rect bounds;
      SDL_GetDisplayBounds(m_screen - 1, &bounds);
      m_win = SDL_CreateWindow("Realtime Video Player",
			       bounds.x, bounds.y, bounds.w, bounds.h,
			       SDL_WINDOW_BORDERLESS);
    } else {
      m_win = SDL_CreateWindow("Realtime Video Player", SDL_WINDOWPOS_UNDEFINED,
			       SDL_WINDOWPOS_UNDEFINED, width, height,
			       (m_fullscreen ? SDL_WINDOW_FULLSCREEN : 0));
    }

    if (!m_win) {
      std::cerr << "Cound not create the SDL window" << std::endl;
      return;
    }
  }

  // Create the renderer
  if (!m_renderer) {
    m_renderer = SDL_CreateRenderer(m_win, -1, 0);
    if (!m_renderer) {
      SDL_DestroyWindow(m_win);
      //std::cerr << "Cound not create the SDL renderer" << std::endl;
      return;
    }
    if (m_screen_width == 0) {
      m_screen_width = width;
      m_screen_height = height;
    }
  }

  // Allocate a place to put our YUV image on that screen
  if (!m_texture) {
    m_texture = SDL_CreateTexture(m_renderer, SDL_PIXELFORMAT_YV12,
				  SDL_TEXTUREACCESS_STREAMING, width, height);
    if (!m_texture) {
      SDL_DestroyRenderer(m_renderer);
      SDL_DestroyWindow(m_win);
      std::cerr << "Could not create the SDL texture" << std::endl;
      return;
    }
  }

  // Create the OSD class
  if (!m_osd) {
    m_osd.reset(new SDLOSD(m_font_file, m_home_dir_icon, m_north_arrow_icon, m_renderer,
			   m_telem, m_screen_width, m_screen_height));
  }

  // Update the texture
  int uv_pitch = width / 2;
  SDL_UpdateYUVTexture(m_texture, NULL, y_plane, width, u_plane, uv_pitch, v_plane, uv_pitch);

  // Update the OSD textures
  m_osd->update();

  // Render the texture
  SDL_RenderClear(m_renderer);
  SDL_RenderCopy(m_renderer, m_texture, NULL, NULL);
  // Draw the OSD textures
  m_osd->draw();
  SDL_RenderPresent(m_renderer);
}
