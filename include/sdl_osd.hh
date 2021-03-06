
#include <string>

#include <SDL.h>
#include <SDL_ttf.h>

#include "texture.hh"
#include "telemetry.hh"

class SDLOSD {
public:

  SDLOSD(const std::string &font_file, const std::string &image_directory, SDL_Renderer *renderer,
	 std::shared_ptr<Telemetry> telem, uint32_t display_width, uint32_t display_display_height);

  void update();

  void draw();

private:
  bool add_text(const std::string &text, TTF_Font *font, TTF_Font *bg_font,
		const SDL_Color &color, const SDL_Color &bg_color, int x, int y,
		SDL_Rect &text_rect);

  bool add_text(const std::string &text, const std::string &units,
		int8_t col, int8_t row, bool from_center,
		uint8_t r = 255, uint8_t g = 255, uint8_t b = 255, uint8_t a = 255,
		uint8_t rb = 0, uint8_t gb = 0, uint8_t bb = 0, uint8_t ab = 0);

  void add_telemetry(const std::string &key, const std::string &format,
		     const std::string &units, int32_t x, int32_t y, bool from_center,
		     uint8_t r = 255, uint8_t g = 255, uint8_t b = 255, uint8_t a = 255,
		     uint8_t rb = 0, uint8_t gb = 0, uint8_t bb = 0, uint8_t ab = 0);

  uint8_t m_text_border; // The width of the border around text in pixels
  uint8_t m_num_lines;
  uint8_t m_num_cols;
  uint32_t m_scaled_screen_height;
  SDL_Renderer *m_renderer;
  Texture *m_home_arrow;
  Texture *m_north_arrow;
  Texture *m_attitude_fg;
  Texture *m_attitude_ground;
  Texture *m_attitude_ring;
  TTF_Font *m_font;
  TTF_Font *m_font_bg;
  TTF_Font *m_units_font;
  TTF_Font *m_units_font_bg;
  std::vector<SDL_Texture*> m_text_textures;
  std::vector<SDL_Rect> m_text_rects;
  std::shared_ptr<Telemetry> m_telem;
};
