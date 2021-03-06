#ifndef LTexture_H_
#define LTexture_H_

#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <SDL2_gfxPrimitives.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include "common.h"

extern SDL_Renderer** globgRenderer;	//	in order to simpilfy the functions, we only use one window anyway...

class LTexture
{
	public:
		//Initializes variables
		LTexture();

		//Deallocates memory
		~LTexture();

		//Loads image at specified path
		bool loadFromFile( std::string path);

		//Creates image from font string
		bool loadFromRenderedText( std::string textureText, SDL_Color textColor, TTF_Font *gFont);

		//Deallocates texture
		void free();

		//Set color modulation
		void setColor( Uint8 red, Uint8 green, Uint8 blue );

		//Set blending
		void setBlendMode( SDL_BlendMode blending );

		//Set alpha modulation
		void setAlpha( Uint8 alpha );

		//Renders texture at given point
		void render( int x, int y, double angle = 0.0, SDL_Rect* clip = NULL, SDL_Point* center = NULL, SDL_RendererFlip flip = SDL_FLIP_NONE );

		//Custom built rendering functions
		void renderRelToScrnRel2Object(double x, double y, LTexture& obj2, double angle = 0.0);
		void renderRelToScrn(double x, double y, double angle = 0.0);
		void renderTXTRelToScrn(double x, double y, double angle = 0.0);

		//Gets image dimensions
		int getWidth();
		int getHeight();

	private:
		//The actual hardware texture
		SDL_Texture* mTexture;
		SDL_Renderer** localgRenderer = globgRenderer;

		//Image dimensions
		int mWidth;
		int mHeight;
};


#endif
