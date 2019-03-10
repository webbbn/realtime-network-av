
#include <iostream>

#include <SDL_ttf.h>

#include "sdl_render_window.hh"

SDLRenderWindow::SDLRenderWindow(std::shared_ptr<Telemetry> telem,
				 const std::string &font_file,
				 const std::string &home_dir_icon,
				 const std::string &north_arrow_icon) :
  m_telem(telem), m_font_file(font_file), m_home_dir_icon(home_dir_icon),
  m_north_arrow_icon(north_arrow_icon), m_screen(0), m_renderer(0), m_texture(0) {

  // Initialize SDL for video output.
  if(SDL_Init(SDL_INIT_VIDEO)) {
    std::cerr << "Could not initialize SDL - " << SDL_GetError() << std::endl;
    SDL_Quit();
    exit(1);
  }

  // Initialize SDL_ttf library
  if (TTF_Init() != 0) {
    std::cerr << "TTF_Init() Failed: " << TTF_GetError() << std::endl;
    SDL_Quit();
    exit(1);
  }
}

SDLRenderWindow::~SDLRenderWindow() {
  if (m_texture) {
    SDL_DestroyTexture(m_texture);
  }
  if (m_renderer) {
    SDL_DestroyRenderer(m_renderer);
  }
  if (m_screen) {
    SDL_DestroyWindow(m_screen);
  }
}

bool SDLRenderWindow::good() const {
  return m_screen && m_texture && m_renderer;
}

bool SDLRenderWindow::check_for_quit() {

  // Check for the SDL quit event.
  SDL_Event event;
  SDL_PollEvent(&event);
  if (event.type == SDL_QUIT) {
    return true;
  }
  return false;
}

void SDLRenderWindow::update(uint32_t width, uint32_t height, uint8_t *y_plane, uint8_t *u_plane, uint8_t *v_plane) {

  // Create the playback window.
  if (!m_screen) {
    m_screen = SDL_CreateWindow("Realtime Video Player", SDL_WINDOWPOS_UNDEFINED,
				SDL_WINDOWPOS_UNDEFINED, width, height, 0);
    if (!m_screen) {
      std::cerr << "Cound not create the SDL window" << std::endl;
      return;
    }
  }

  // Create the renderer
  if (!m_renderer) {
    m_renderer = SDL_CreateRenderer(m_screen, -1, 0);
    if (!m_renderer) {
      SDL_DestroyWindow(m_screen);
      std::cerr << "Cound not create the SDL renderer" << std::endl;
      return;
    }

    // Scale the renderer based on a 1280x720 screen size
    float scale = static_cast<float>(width) / 1280.0;
    SDL_RenderSetScale(m_renderer, scale, scale);
  }

  // Allocate a place to put our YUV image on that screen
  if (!m_texture) {
    m_texture = SDL_CreateTexture(m_renderer, SDL_PIXELFORMAT_YV12,
				  SDL_TEXTUREACCESS_STREAMING, width, height);
    if (!m_texture) {
      SDL_DestroyRenderer(m_renderer);
      SDL_DestroyWindow(m_screen);
      std::cerr << "Could not create the SDL texture" << std::endl;
      return;
    }
  }

  // Create the OSD class
  if (!m_osd) {
    m_osd.reset(new SDLOSD(m_font_file, m_home_dir_icon, m_north_arrow_icon, m_renderer,
			   m_telem, width, height));
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
