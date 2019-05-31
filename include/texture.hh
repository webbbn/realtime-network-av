#ifndef RTNAV_TEXTURE_HH
#define RTNAV_TEXTURE_HH

#include <SDL.h>

//Texture wrapper class
class Texture {
public:

  // Initializes variables
  Texture(SDL_Renderer *renderer);

  // Deallocates memory
  ~Texture();

  // Loads image at specified path
  bool load_from_file(const std::string &path);

  // Deallocates texture
  void free();

  // Set color modulation
  void set_color(uint8_t red, uint8_t green, uint8_t blue);

  // Set blending
  void set_blendMode(SDL_BlendMode blending);

  // Set alpha modulation
  void set_alpha(uint8_t alpha);
        
  //Renders texture at given point
  void render(int x, int y, SDL_Rect* clip = 0, double angle = 0.0, SDL_Point* center = NULL,
	      uint8_t alphamod = 255, SDL_RendererFlip flip = SDL_FLIP_NONE);

  //Gets image dimensions
  int width();
  int height();

private:
  // The renderer
  SDL_Renderer *m_renderer;

  //The actual hardware texture
  SDL_Texture *m_texture;

  //Image dimensions
  int m_width;
  int m_height;
};

#endif /* RTNAV_TEXTURE */
