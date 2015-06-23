/*This source code copyrighted by Lazy Foo' Productions (2004-2015)
  and may not be redistributed without written permission.*/

//Using SDL, SDL_image, SDL_ttf, standard IO, math, and strings
#include "gui.h"
#include "sensorenv.h"
#include <time.h>
#include <iostream>
#include <vector>
#include "common.h"

extern char sensors_buf[BUF_SIZE], bt_buf[BUF_SIZE], gps_buf[BUF_SIZE] ,velocity_buf[BUF_SIZE];
extern VnDeviceCompositeData sensorData;
extern bool globQuitSig;
int velocity = 0;
int gear = 0;
int RPM = 0;

std::vector<Point> pts;
std::vector<Point> Updated_pts;

//The window we'll be rendering to
SDL_Window* gWindow = NULL;

//The window renderer
SDL_Renderer* gRenderer = NULL;
SDL_Renderer** globgRenderer = &gRenderer;

//Globally used font
TTF_Font *gFont = NULL, *gDigitalFont = NULL;

//text for the velocity and gear.
LTexture gTextVelocity, gTextGear;

//creating the gear and the velocity gradients.
LTexture gVelocityGradient;
LTexture gGearGradient[6];
LTexture gNeedleTexture;

//Artificial Horizon
LTexture gArtHorzTexture;

bool init()
{
	//Initialization flag
	bool success = true;

	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
	{
		printf( "SDL could not initialize! SDL Error: %s\n", SDL_GetError() );
		success = false;
	}
	else
	{

		//Set texture filtering to linear
		if( !SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" ) )
		{
			printf( "Warning: Linear texture filtering not enabled!" );
		}

		//Create window
		gWindow = SDL_CreateWindow( "HUD", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
		if( gWindow == NULL )
		{
			printf( "Window could not be created! SDL Error: %s\n", SDL_GetError() );
			success = false;
		}
		else
		{
			//Create vsynced renderer for window
			gRenderer = SDL_CreateRenderer( gWindow, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC );
			if( gRenderer == NULL )
			{
				printf( "Renderer could not be created! SDL Error: %s\n", SDL_GetError() );
				success = false;
			}
			else
			{
				//Initialize renderer color
				SDL_SetRenderDrawColor( gRenderer, 0, 0, 0, 0 );

				//Initialize PNG loading
				int imgFlags = IMG_INIT_PNG;
				if( !( IMG_Init( imgFlags ) & imgFlags ) )
				{
					printf( "SDL_image could not initialize! SDL_image Error: %s\n", IMG_GetError() );
					success = false;
				}

				//Initialize SDL_ttf
				if( TTF_Init() == -1 )
				{
					printf( "SDL_ttf could not initialize! SDL_ttf Error: %s\n", TTF_GetError() );
					success = false;
				}
			}
		}
	}
	return success;
}

bool loadMedia()
{
	//Open the font
	gDigitalFont = TTF_OpenFont( PROJ_HOME "/resources/digital-7.ttf", FONT_7_SIZE);
	if( !gDigitalFont ) {
		printf( "Failed to load lazy font! SDL_ttf Error: %s\n", TTF_GetError() );
		return false;
	}
	//Load media
	return 	gGearGradient[0].loadFromFile( PROJ_HOME "/resources/gearFinal1.png" ) 		&&
		gGearGradient[1].loadFromFile( PROJ_HOME "/resources/gearFinal2.png" ) 		&&
		gGearGradient[2].loadFromFile( PROJ_HOME "/resources/gearFinal3.png" ) 		&&
		gGearGradient[3].loadFromFile( PROJ_HOME "/resources/gearFinal4.png" ) 		&&
		gGearGradient[4].loadFromFile( PROJ_HOME "/resources/gearFinal5.png" ) 		&&
		gGearGradient[5].loadFromFile( PROJ_HOME "/resources/gearFinal6.png" ) 		&&
		gVelocityGradient.loadFromFile( PROJ_HOME "/resources/velocityGradient.png" ) &&
		gNeedleTexture.loadFromFile( PROJ_HOME "/resources/needle-fioptics2.png" ) 	&&
		gArtHorzTexture.loadFromFile( PROJ_HOME "/resources/artHorz.png" ) 		&&
		gTextVelocity.loadFromRenderedText( "Tadaa", VEL_FONT_COLOR,gDigitalFont )	&&
		gTextGear.loadFromRenderedText( "Tadaa", GEAR_FONT_COLOR, gDigitalFont );
}


bool reloadText()
{
	int velocityInt = velocity;
	int gearInt = gear;

	std::string strVelocity = std::to_string(velocityInt);
	std::string strGear = std::to_string(gearInt);

	//Render text
	return 	gTextVelocity.loadFromRenderedText(strVelocity, VEL_FONT_COLOR,gDigitalFont) &&
		gTextGear.loadFromRenderedText(strGear, GEAR_FONT_COLOR,gDigitalFont);
}

void close()
{

	//Free loaded images
	gGearGradient[0].free();
	gGearGradient[1].free();
	gGearGradient[2].free();
	gGearGradient[3].free();
	gGearGradient[4].free();
	gGearGradient[5].free();
	gVelocityGradient.free();
	gNeedleTexture.free();
	gArtHorzTexture.free();

	//Free global font
	TTF_CloseFont( gFont );
	TTF_CloseFont(gDigitalFont);
	gFont = NULL;

	//Destroy window
	SDL_DestroyRenderer( gRenderer );
	SDL_DestroyWindow( gWindow );
	gWindow = NULL;
	gRenderer = NULL;

	//Quit SDL subsystems
	TTF_Quit();
	IMG_Quit();
	SDL_Quit();
}

void* gui_main(void* arg)
{
	short degrees = 0;
	double horDeg = 0;
	guiUtils utils;
	//double yawDeg = 0;
	int numFrames = 0;

#ifdef TRACK_FPS
	Uint32 startTime = SDL_GetTicks();
	float fps = 0;
#endif
	int velocityInt = 0;
	int gearInt = 0;
	int RPMint = 0;

	std::string strVelocity = std::to_string(velocityInt);
	std::string strGear = std::to_string(gearInt);

	/**** Points for show***/
	pts.push_back(Point(200,200));
	pts.push_back(Point(240,280));
	pts.push_back(Point(280,360));
	pts.push_back(Point(320,400));
	pts.push_back(Point(360,420));
	pts.push_back(Point(400,420));
	pts.push_back(Point(440,400));
	pts.push_back(Point(480,360));
	pts.push_back(Point(440,280));
	pts.push_back(Point(400,240));
	pts.push_back(Point(360,200));
	pts.push_back(Point(320,180));
	pts.push_back(Point(280,150));
	pts.push_back(Point(240,100));
	pts.push_back(Point(200,130));
	pts.push_back(Point(200,150));
	pts.push_back(Point(200,180));
	pts.push_back(Point(200,200));


	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	Updated_pts.push_back(Point(0,0));
	/*****/
	std::vector<short> vel;

	uint ptsIndex = 0;

	std::vector<Point> pts_Updated;

	//Start up SDL and create window
	if( !init() ) {
		printf( "Failed to initialize!\n" );
	} else {
		//Load media
		if( !loadMedia() ) {
			printf( "Failed to load media!\n" );
		} else {
			//Event handler
			SDL_Event e;

			//While application is running
			while( !globQuitSig ) {
				//Handle events on queue
				while( SDL_PollEvent( &e ) != 0 ) {
					//User requests globQuitSig
					if( e.type == SDL_QUIT ) {
						globQuitSig = true;
					}
				}

				// for demo only. to be connected to the actual values
				numFrames++;
				velocityInt += VELOCITY_STEP;
				RPMint      += RPM_STEP;
				gearInt     += GEAR_STEP;
				velocityInt %= MAX_VELOCITY;
				RPMint      %= MAX_RPM;
				gearInt     %= MAX_GEAR;

				degrees     = (velocityInt*MAX_DEGREE)/MAX_VELOCITY + DEGREES_OFFSET;

				// connecting with the global variables
				RPM = RPMint;
				velocity = velocityInt;
				gear = gearInt;

#ifdef TRACK_FPS
				fps = ( numFrames/(float)(SDL_GetTicks() - startTime) )*1000;
				printf("FPS: %lf\n", fps);
#endif

				horDeg = (double)sensorData.ypr.pitch;

				//Clear screen
				SDL_SetRenderDrawColor( gRenderer, 0, 0, 0, 0 );	//	background screen color
				SDL_RenderClear( gRenderer );

				//Render current frame
				//pngs
				gGearGradient[(RPMint*6)/MAX_RPM].renderRelToScrn(RELATIVE_PLACE_RPM_X, RELATIVE_PLACE_RPM_Y );
				gArtHorzTexture.renderRelToScrn(RELATIVE_PLACE_ARTHORZ_X, RELATIVE_PLACE_ARTHORZ_Y, 0.0+horDeg);
				gVelocityGradient.renderRelToScrn(RELATIVE_PLACE_VELOCITY_G_X, RELATIVE_PLACE_VELOCITY_G_Y);
				gNeedleTexture.renderRelToScrnRel2Object(RELATIVE_PLACE_SPEEDOMETER_X, RELATIVE_PLACE_SPEEDOMETER_Y, gVelocityGradient, -50.0+degrees);

				//txts
				reloadText();
				gTextGear.renderTXTRelToScrn(RELATIVE_PLACE_FONT_GEAR_X, RELATIVE_PLACE_FONT_GEAR_Y);
				gTextVelocity.renderTXTRelToScrn(RELATIVE_PLACE_FONT_VELOCITY_X, RELATIVE_PLACE_FONT_VELOCITY_Y);

				Point p(300,300);
				Point delXY(-80,-50);
				//utils.rotateVec(pts , Updated_pts , p , 20);
				//Point delXY(-100,-100);
				utils.translateVec(pts ,Updated_pts ,delXY);
				//utils.strechVec(pts ,Updated_pts, p ,0.5, 'y');
				//utils.strechVec(pts ,Updated_pts, p ,0.2, 'x');
				for(ptsIndex = 1; ptsIndex <= pts.size() ;ptsIndex++)
				{
					thickLineRGBA(gRenderer ,Updated_pts[ptsIndex-1].X ,Updated_pts[ptsIndex-1].Y ,Updated_pts[ptsIndex % pts.size()].X ,Updated_pts[ptsIndex % pts.size()].Y,10 ,100,100,255,155);
				}
				//Update screen
				SDL_RenderPresent( gRenderer );
			}
		}
	}

	//Free resources and close SDL
	close();

	return 0;
}

