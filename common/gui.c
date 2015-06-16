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

std::vector<Point> pts;
std::vector<Point> Updated_pts;


//The window we'll be rendering to
SDL_Window* gWindow = NULL;

//The window renderer
SDL_Renderer* gRenderer = NULL;

//Globally used font
TTF_Font *gFont = NULL;

//Rendered texture
LTexture gTextTextureBT, gTextTextureSens , gTextTextureGPS;

//Current displayed the speedometer background, needle &RPM
LTexture gSpeedometerBackgroundTexture,gNeedleTexture ,gRPMTexture ,gRPMNeedleTexture;

//for the track, use polygon.
LTexture gPolygon;

//Artificial Horizon
LTexture gArtHorzTexture;

int RelativePosition1Object(int screenLength, int objectLentgh, double relativePosition)
{
	return ((screenLength - objectLentgh)*relativePosition);
}
int MidPositionRelative(int screenLength,int object1Lentgh,int object2Length,double relativePosition)
{
	return ((screenLength - object1Lentgh)*relativePosition) + object1Lentgh/2.0 -object2Length/2.0;
}
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
	//Loading success flag
	bool success = true;

	//Open the font
	gFont = TTF_OpenFont( "/usr/share/fonts/truetype/ubuntu-font-family/UbuntuMono-R.ttf", 28 );
	if( gFont == NULL )
	{
		printf( "Failed to load lazy font! SDL_ttf Error: %s\n", TTF_GetError() );
		success = false;
	}
	else
	{
		//Render text
		SDL_Color textColor = { 0xFF, 0xFF, 0xFF };
		if( !gTextTextureBT.loadFromRenderedText( "Tadaa", textColor,gFont,gRenderer ) || !gTextTextureSens.loadFromRenderedText( "Tadaa", textColor, gFont,gRenderer ))
		{
			printf( "Failed to render text texture!\n" );
			success = false;
		}
	}
	if( !gSpeedometerBackgroundTexture.loadFromFile( PROJ_HOME "/resources/step25.gif",gRenderer ) )
	{
		printf( "Failed to load speedometer backdround image - texture!\n" );
		success = false;
	}
	if( !gNeedleTexture.loadFromFile( PROJ_HOME "/resources/needle-fioptics2.png",gRenderer ) )
	{
		printf( "Failed to load speedometer backdround image - texture!\n" );
		success = false;
	}

	if( !gRPMTexture.loadFromFile( PROJ_HOME "/resources/lfa-rpm3.gif",gRenderer ) )
	{
		printf( "Failed to load rpm backdround image - texture!\n" );
		success = false;
	}
	if( !gRPMNeedleTexture.loadFromFile( PROJ_HOME "/resources/needle-fiopticsRPM3.png",gRenderer ) )
	{
		printf( "Failed to load rpm needle image - texture!\n" );
		success = false;
	}

	if( !gArtHorzTexture.loadFromFile( PROJ_HOME "/resources/artHorz.gif",gRenderer ) )
	{
		printf( "Failed to load artificial horizon image - texture!\n" );
		success = false;
	}


	return success;
}


bool reloadText()
{
	bool success = true;

	//Render text
	SDL_Color textColor = { 0xFF, 0xFF, 0xFF };

	if(!sensors_buf[0]) {
		if( !gTextTextureSens.loadFromRenderedText( "Sensors buffer empty", textColor, gFont,gRenderer) ) {
			printf( "Failed to render text texture!\n" );
			success = false;
		}
	} else {
		if( !gTextTextureSens.loadFromRenderedText( sensors_buf, textColor,gFont,gRenderer ) ) {
			printf( "Failed to render sensors text texture!\n" );
			success = false;
		}
	}

	//GPS
	if(!gps_buf[0]) {
		if( !gTextTextureGPS.loadFromRenderedText( "Sensors buffer empty", textColor, gFont,gRenderer) ) {
			printf( "Failed to render text texture!\n" );
			success = false;
		}
	} else {
		if( !gTextTextureGPS.loadFromRenderedText( gps_buf, textColor,gFont,gRenderer ) ) {
			printf( "Failed to render GPS text texture!\n" );
			success = false;
		}
	}



	if(!bt_buf[0]) {
		if( !gTextTextureBT.loadFromRenderedText( "Bluetooth buffer empty", textColor,gFont,gRenderer ) ) {
			printf( "Failed to render text texture!\n" );
			success = false;
		}
	} else {
		if( !gTextTextureBT.loadFromRenderedText( bt_buf, textColor,gFont,gRenderer ) ) {
			printf( "Failed to render bluetooth text texture!\n" );
			success = false;
		}
	}

	return success;
}

void close()
{
	//Free loaded images
	gTextTextureBT.free();
	gTextTextureSens.free();
	gTextTextureGPS.free();
	gSpeedometerBackgroundTexture.free();
	gNeedleTexture.free();
	gRPMTexture.free();
	gRPMNeedleTexture.free();
	gArtHorzTexture.free();

	//Free global font
	TTF_CloseFont( gFont );
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
	//double yawDeg = 0;
	int numFrames = 0;
	Uint32 startTime = SDL_GetTicks();
 	float fps = 0;
	pts.push_back(Point(200,200));
	pts.push_back(Point(300,400));
	pts.push_back(Point(400,200));
	uint ptsIndex = 0;

	//std::vector<Point> pts_Updated;

	//Start up SDL and create window
	if( !init() )
	{
		printf( "Failed to initialize!\n" );
	}
	else
	{
		//Load media
		if( !loadMedia() )
		{
			printf( "Failed to load media!\n" );
		}
		else
		{
			//Event handler
			SDL_Event e;

			//While application is running
			while( !globQuitSig )
			{
				//Handle events on queue
				while( SDL_PollEvent( &e ) != 0 )
				{
					//User requests globQuitSig
					if( e.type == SDL_QUIT )
					{
						globQuitSig = true;
					}
				}
				numFrames++;
				degrees++;
				fps = ( numFrames/(float)(SDL_GetTicks() - startTime) )*1000;
				//printf("FPS: %lf\n", fps);
				horDeg = (double)sensorData.ypr.roll;
				



				//Clear screen
				SDL_SetRenderDrawColor( gRenderer, 0, 0, 0, 0 );	//	background screen color
				SDL_RenderClear( gRenderer );


				//Render current frame
				gSpeedometerBackgroundTexture.render(
						RelativePosition1Object(SCREEN_WIDTH,	gSpeedometerBackgroundTexture.getWidth(),	RELATIVE_PLACE_SPEEDOMETER_X),
						RelativePosition1Object(SCREEN_HEIGHT,	gSpeedometerBackgroundTexture.getHeight(),	RELATIVE_PLACE_SPEEDOMETER_Y),
						gRenderer );
				gRPMTexture.render(
						RelativePosition1Object(SCREEN_WIDTH,	gRPMTexture.getWidth(),		RELATIVE_PLACE_RPM_X),
						RelativePosition1Object(SCREEN_HEIGHT,	gRPMTexture.getHeight(),	RELATIVE_PLACE_RPM_Y),
						gRenderer );
				gNeedleTexture.render(
						MidPositionRelative(SCREEN_WIDTH,	gSpeedometerBackgroundTexture.getWidth(),	gNeedleTexture.getWidth(),	RELATIVE_PLACE_SPEEDOMETER_X),
						MidPositionRelative(SCREEN_HEIGHT,	gSpeedometerBackgroundTexture.getHeight(), 	gNeedleTexture.getHeight(),	RELATIVE_PLACE_SPEEDOMETER_Y),
						gRenderer, NULL, -50.0+degrees, NULL, SDL_FLIP_NONE );
				gRPMNeedleTexture.render(
						MidPositionRelative(SCREEN_WIDTH,	gRPMTexture.getWidth(),		gRPMNeedleTexture.getWidth(),	RELATIVE_PLACE_RPM_X),
						MidPositionRelative(SCREEN_HEIGHT,	gRPMTexture.getHeight(),	gRPMNeedleTexture.getHeight(), 	RELATIVE_PLACE_RPM_Y) ,
						gRenderer, NULL, -30.0+degrees, NULL, SDL_FLIP_NONE );
				gArtHorzTexture.render(
						MidPositionRelative(SCREEN_WIDTH,	gArtHorzTexture.getWidth(),	gArtHorzTexture.getWidth(),	RELATIVE_PLACE_ARTHORZ_X),
						MidPositionRelative(SCREEN_HEIGHT,	gArtHorzTexture.getHeight(), 	gArtHorzTexture.getHeight(),	RELATIVE_PLACE_ARTHORZ_Y),
						gRenderer, NULL, 0.0+horDeg, NULL, SDL_FLIP_NONE );

				reloadText();
				gTextTextureBT.render( ( SCREEN_WIDTH - gTextTextureBT.getWidth() ) / 2, ( SCREEN_HEIGHT - gTextTextureBT.getHeight() ) / 10,gRenderer );
				gTextTextureSens.render( ( SCREEN_WIDTH - gTextTextureSens.getWidth() ) / 2, ( SCREEN_HEIGHT - gTextTextureSens.getHeight() ) * 9 / 10,gRenderer );
				gTextTextureGPS.render( ( SCREEN_WIDTH - gTextTextureSens.getWidth() ) * 7 / 8, ( SCREEN_HEIGHT - gTextTextureGPS.getHeight() ) * 3 / 10,gRenderer );

				//load the polygon:
				//Try to rotate according to Yaw
				//rotatePts(Xtrack, Ytrack, n, 0.25 , 200,200 ,Xtrack_Updated ,Ytrack_Updated);
				//shiftPTS(Xtrack,Ytrack,n,Xtrack[(degrees/10+1)%n]-Xtrack[degrees/10%n],Ytrack[(degrees/10+1)%n]-Ytrack[degrees/10%n],Xtrack_Updated, 	Ytrack_Updated);
				
				
				//if (polygonRGBA(gRenderer,Xtrack_Updated, Ytrack_Updated,n,255, 255, 255, 155))
					//printf("Priel Ya Manyak!");

				for(ptsIndex = 1; ptsIndex <= pts.size() ;ptsIndex++)
				{				
					lineRGBA(gRenderer ,pts[ptsIndex-1].X ,pts[ptsIndex-1].Y ,pts[ptsIndex % pts.size()].X ,pts[ptsIndex % pts.size()].Y ,100,100,255,155);
					//ptsUpdated.push_back(pts[ptsIndex]);	
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

