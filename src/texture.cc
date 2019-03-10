
#include <iostream>

#include <SDL_image.h>

#include "texture.hh"

Texture::Texture(SDL_Renderer *renderer) : m_renderer(renderer) {
  //Initialize
  m_texture = NULL;
  m_width = 0;
  m_height = 0;
}

Texture::~Texture() {
  //Deallocate
  free();
}

bool Texture::load_from_file(const std::string &path) {
  //Get rid of preexisting texture
  free();

  //The final texture
  SDL_Texture* new_texture = NULL;

  //Load image at specified path
  SDL_Surface* loaded_surface = IMG_Load(path.c_str());
  if(!loaded_surface) {
    std::cerr << "Unable to load image " << path << "! SDL_image Error: " << IMG_GetError()
	      << std::endl;
  } else {
    // Color key image
    SDL_SetColorKey(loaded_surface, SDL_TRUE, SDL_MapRGB(loaded_surface->format, 0, 0xFF, 0xFF));

    //Create texture from surface pixels
    new_texture = SDL_CreateTextureFromSurface(m_renderer, loaded_surface);
    if(new_texture == NULL) {
      std::cerr << "Unable to create texture from " << path << " SDL Error: " << SDL_GetError()
		<< std::endl;
    } else {
      //Get image dimensions
      m_width = loaded_surface->w;
      m_height = loaded_surface->h;
    }

    //Get rid of old loaded surface
    SDL_FreeSurface(loaded_surface);
  }

    //Return success
  m_texture = new_texture;
  return m_texture != NULL;
}

void Texture::free() {
  //Free texture if it exists
  if(m_texture != NULL) {
    SDL_DestroyTexture(m_texture);
    m_texture = NULL;
    m_width = 0;
    m_height = 0;
  }
}

void Texture::render(int x, int y, SDL_Rect* clip, double angle, SDL_Point* center,
		     SDL_RendererFlip flip) {
  // Set rendering space and render to screen
  SDL_Rect render_quad = { x, y, m_width, m_height };

  // Set clip rendering dimensions
  if (clip != NULL) {
    render_quad.w = clip->w;
    render_quad.h = clip->h;
  }

  // Render to screen
  SDL_RenderCopyEx(m_renderer, m_texture, clip, &render_quad, angle, center, flip);
}

int Texture::width() {
  return m_width;
}

int Texture::height() {
  return m_height;
}
