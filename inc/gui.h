#ifndef GUI_H_
#define GUI_H_

#include "LTexture.h"
#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <SDL2_gfxPrimitives.h>
#include <SDL_mixer.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include "common.h"

//Screen dimension constants
#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 720

// fraction that will represent the relative place of the speedometer relative in the screen
#define RELATIVE_PLACE_SPEEDOMETER_X -1.0/10 // we must use .0 so it will considered as double.
#define RELATIVE_PLACE_SPEEDOMETER_Y 10.0/10

// fraction bla bla bla what priel wrote...
#define RELATIVE_PLACE_RPM_X 10.8/10
#define RELATIVE_PLACE_RPM_Y 10.0/10

//Artificial Horizon
#define RELATIVE_PLACE_ARTHORZ_X 5.0/10
#define RELATIVE_PLACE_ARTHORZ_Y 10.0/10



//place relative position
int MidPositionRelative(int screenLength,int object1Lentgh,int object2Length,double relativePosition);
int RelativePosition1Object(int screenLength, int objectLentgh, double relativePosition);



//Starts up SDL and creates window
bool init();

//Loads media
bool loadMedia();

//Frees media and shuts down SDL
void close();

//Reloads text from buffer
bool reloadText();

//optimize image surface
SDL_Surface*  loadSurfaceOptimalImg( std::string path );



void* gui_main(void* arg);


#endif
