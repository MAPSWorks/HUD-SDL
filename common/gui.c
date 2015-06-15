/*This source code copyrighted by Lazy Foo' Productions (2004-2015)
  and may not be redistributed without written permission.*/

//Using SDL, SDL_image, SDL_ttf, standard IO, math, and strings
#include "gui.h"
#include "sensorenv.h"
#include <time.h>



extern char sensors_buf[BUF_SIZE], bt_buf[BUF_SIZE], gps_buf[BUF_SIZE] ,velocity_buf[BUF_SIZE];
extern VnDeviceCompositeData sensorData;

//here is a dummy x,y point to illustrate the track, the track is poligon of all points.
int n = 3; // size of the array.
short Xtrack[3] = {200 ,300 ,400};
short Ytrack [3] = {200, 400 ,200};

short Xtrack_Updated[3] = {0};
short Ytrack_Updated[3] = {0};

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
	if( !gSpeedometerBackgroundTexture.loadFromFile( "/home/odroid/project/resources/step25.gif",gRenderer ) )
	{
		printf( "Failed to load speedometer backdround image - texture!\n" );
		success = false;
	}
	if( !gNeedleTexture.loadFromFile( "/home/odroid/project/resources/needle-fioptics2.png",gRenderer ) )
	{
		printf( "Failed to load speedometer backdround image - texture!\n" );
		success = false;
	}

	if( !gRPMTexture.loadFromFile( "/home/odroid/project/resources/lfa-rpm3.gif",gRenderer ) )
	{
		printf( "Failed to load rpm backdround image - texture!\n" );
		success = false;
	}
	if( !gRPMNeedleTexture.loadFromFile( "/home/odroid/project/resources/needle-fiopticsRPM3.png",gRenderer ) )
	{
		printf( "Failed to load rpm needle image - texture!\n" );
		success = false;
	}

	if( !gArtHorzTexture.loadFromFile( "/home/odroid/project/resources/artHorz.gif",gRenderer ) )
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
	double degrees =0;
	double horDeg = 0;
	double yawDeg = 0;

	time_t mytime = time(NULL);
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
			//Main loop flag
			bool quit = false;

			//Event handler
			SDL_Event e;

			//While application is running
			while( !quit )
			{
				//Handle events on queue
				while( SDL_PollEvent( &e ) != 0 )
				{
					//User requests quit
					if( e.type == SDL_QUIT )
					{
						quit = true;
					}
				}
				degrees +=1;
				horDeg = (double)sensorData.ypr.roll;
				yawDeg = (double)sensorData.ypr.yaw;
				
				printf("refresh rate is: %lf\n",(1/(time(NULL) - mytime)));
				mytime = time(NULL);



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
				//rotatePts(Xtrack, Ytrack, n, 2, 300,300 ,Xtrack_Updated ,Ytrack_Updated);
				//if (!polygonRGBA(gRenderer,Xtrack, Ytrack,n,255, 255, 255, 155))
				//	printf("failed to render the polygon");
				//Update screen
				SDL_RenderPresent( gRenderer );
			}
		}
	}

	//Free resources and close SDL
	close();

	return 0;
}

