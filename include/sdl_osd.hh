
#include <string>

#include <SDL.h>
#include <SDL_ttf.h>

#include "texture.hh"
#include "telemetry.hh"

class SDLOSD {
public:

  SDLOSD(const std::string &font_file, const std::string &home_dir_icon,
	 const std::string &north_arrow, SDL_Renderer *renderer,
	 std::shared_ptr<Telemetry> telem,
	 uint32_t display_width, uint32_t display_display_height);

  void update();

  void draw();

private:
  bool add_text(const std::string &text, TTF_Font *font, TTF_Font *bg_font,
		const SDL_Color &color, const SDL_Color &bg_color, int x, int y,
		SDL_Rect &text_rect);

  bool add_text(const std::string &text, const std::string &units,
		uint8_t col, uint8_t row,
		uint8_t r = 255, uint8_t g = 255, uint8_t b = 255, uint8_t a = 255,
		uint8_t rb = 0, uint8_t gb = 0, uint8_t bb = 0, uint8_t ab = 0);

  void add_telemetry(const std::string &key, const std::string &format, const std::string &units,
		     uint32_t x, uint32_t y);

  uint8_t m_text_border; // The width of the border around text in pixels
  SDL_Renderer *m_renderer;
  Texture *m_home_arrow;
  Texture *m_north_arrow;
  TTF_Font *m_font;
  TTF_Font *m_font_bg;
  TTF_Font *m_units_font;
  TTF_Font *m_units_font_bg;
  std::vector<SDL_Texture*> m_text_textures;
  std::vector<SDL_Rect> m_text_rects;
  std::shared_ptr<Telemetry> m_telem;
};
