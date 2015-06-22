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

//Globally used font
TTF_Font *gFont = NULL, *gDigitalFont = NULL;

//text for the velocity and gear.
LTexture gTextVelocity, gTextGear;


//creating the gear and the velocity gradients.
LTexture gGearGradient1, gGearGradient2, gGearGradient3, gGearGradient4, gGearGradient5, gGearGradient6, gVelocityGradient;

LTexture gNeedleTexture;

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

	gDigitalFont = TTF_OpenFont( PROJ_HOME "/resources/digital-7.ttf", FONT_7_SIZE);
	if( gDigitalFont == NULL )
	{
		printf( "Failed to load lazy font! SDL_ttf Error: %s\n", TTF_GetError() );
		success = false;
	}
	else
	{
		//Render text
		SDL_Color textColor = { 0xFF, 0xFF, 0xFF };
		if( !gTextVelocity.loadFromRenderedText( "Tadaa", textColor,gDigitalFont,gRenderer ) || !gTextGear.loadFromRenderedText( "Tadaa", textColor, gDigitalFont,gRenderer ))
		{
			printf( "Failed to render text texture!\n" );
			success = false;
		}
	}
	if( !gGearGradient1.loadFromFile( PROJ_HOME "/resources/gearFinal1.png",gRenderer ) )
	{
		printf( "Failed to load Gear1 backdround image - texture!\n" );
		success = false;
    }
    if( !gGearGradient2.loadFromFile( PROJ_HOME "/resources/gearFinal2.png",gRenderer ) )
	{
		printf( "Failed to load Gear2 backdround image - texture!\n" );
		success = false;
    }
    if( !gGearGradient3.loadFromFile( PROJ_HOME "/resources/gearFinal3.png",gRenderer ) )
	{
		printf( "Failed to load Gear3 backdround image - texture!\n" );
		success = false;
    }
    if( !gGearGradient4.loadFromFile( PROJ_HOME "/resources/gearFinal4.png",gRenderer ) )
	{
		printf( "Failed to load Gear4 backdround image - texture!\n" );
		success = false;
    }
    if( !gGearGradient5.loadFromFile( PROJ_HOME "/resources/gearFinal5.png",gRenderer ) )
	{
		printf( "Failed to load Gear5 backdround image - texture!\n" );
		success = false;
    }
    if( !gGearGradient6.loadFromFile( PROJ_HOME "/resources/gearFinal6.png",gRenderer ) )
	{
		printf( "Failed to load Gear6 backdround image - texture!\n" );
		success = false;
    }

    if( !gVelocityGradient.loadFromFile( PROJ_HOME "/resources/velocityGradient.png",gRenderer ) )
	{
		printf( "Failed to load gVelocityGradient backdround image - texture!\n" );
		success = false;
    }



	if( !gNeedleTexture.loadFromFile( PROJ_HOME "/resources/needle-fioptics2.png",gRenderer ) )
	{
		printf( "Failed to load speedometer backdround image - texture!\n" );
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
	int velocityInt = velocity;
    int gearInt = gear;

    std::string strVelocity = std::to_string(velocityInt);
    std::string strGear = std::to_string(gearInt);

	//Render text
	SDL_Color textColor = { 0xFF, 0xFF, 0xFF };

	if(!gTextVelocity.loadFromRenderedText(strVelocity, textColor,gDigitalFont,gRenderer)){
        printf( "failed to load velocity text");
        success = false;
    }
    if(!gTextGear.loadFromRenderedText(strGear, textColor,gDigitalFont,gRenderer)){
        printf( "failed to load Gear text");
        success = false;
    }

	return success;
}

void close()
{

	//Free loaded images
	gGearGradient1.free();
	gGearGradient2.free();
	gGearGradient3.free();
	gGearGradient4.free();
	gGearGradient5.free();
	gGearGradient6.free();
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
    int velocityInt = 0;
    int gearInt = 0;
    int RPMint = 0;

    std::string strVelocity = std::to_string(velocityInt);
    std::string strGear = std::to_string(gearInt);


//	pts.push_back(Point(200,200));
//	pts.push_back(Point(300,400));
//	pts.push_back(Point(400,200));

//	Updated_pts.push_back(Point(0,0));
//	Updated_pts.push_back(Point(0,0));
//	Updated_pts.push_back(Point(0,0));

//	std::vector<short> velocity;

//	uint ptsIndex = 0;

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
				horDeg = (double)sensorData.ypr.roll;



				//Clear screen
				SDL_SetRenderDrawColor( gRenderer, 0, 0, 0, 0 );	//	background screen color
				SDL_RenderClear( gRenderer );

				//Render current frame

				gArtHorzTexture.render(
						MidPositionRelative(SCREEN_WIDTH,	gArtHorzTexture.getWidth(),	gArtHorzTexture.getWidth(),	RELATIVE_PLACE_ARTHORZ_X),
						MidPositionRelative(SCREEN_HEIGHT,	gArtHorzTexture.getHeight(), 	gArtHorzTexture.getHeight(),	RELATIVE_PLACE_ARTHORZ_Y),
						gRenderer, NULL, 0.0+horDeg, NULL, SDL_FLIP_NONE );

                gVelocityGradient.render(
						MidPositionRelative(SCREEN_WIDTH,	gArtHorzTexture.getWidth(),	gArtHorzTexture.getWidth(), 	RELATIVE_PLACE_VELOCITY_G_X),
						MidPositionRelative(SCREEN_HEIGHT,	gArtHorzTexture.getHeight(), 	gArtHorzTexture.getHeight(),	RELATIVE_PLACE_VELOCITY_G_Y),
						gRenderer, NULL, 0.0+horDeg, NULL, SDL_FLIP_NONE );

                gNeedleTexture.render(
						MidPositionRelative(SCREEN_WIDTH,	gGearGradient1.getWidth(),	gNeedleTexture.getWidth(),	RELATIVE_PLACE_SPEEDOMETER_X),
						MidPositionRelative(SCREEN_HEIGHT,	gGearGradient1.getHeight(), gNeedleTexture.getHeight(),	RELATIVE_PLACE_SPEEDOMETER_Y),
						gRenderer, NULL, -50.0+degrees, NULL, SDL_FLIP_NONE );

                switch ((RPMint*6)/MAX_RPM + 1)
                {
                    case 1:
                        gGearGradient1.render(
                            RelativePosition1Object(SCREEN_WIDTH,	gGearGradient1.getWidth(),	RELATIVE_PLACE_RPM_X),
                            RelativePosition1Object(SCREEN_HEIGHT,	gGearGradient1.getHeight(),	RELATIVE_PLACE_RPM_Y),
                            gRenderer );
                        break;
                    case 2:
                        gGearGradient2.render(
                            RelativePosition1Object(SCREEN_WIDTH,	gGearGradient2.getWidth(),	RELATIVE_PLACE_RPM_X),
                            RelativePosition1Object(SCREEN_HEIGHT,	gGearGradient2.getHeight(),	RELATIVE_PLACE_RPM_Y),
                            gRenderer );
                        break;
                    case 3:
                        gGearGradient3.render(
                            RelativePosition1Object(SCREEN_WIDTH,	gGearGradient3.getWidth(),	RELATIVE_PLACE_RPM_X),
                            RelativePosition1Object(SCREEN_HEIGHT,	gGearGradient3.getHeight(),	RELATIVE_PLACE_RPM_Y),
                            gRenderer );
                        break;
                    case 4:
                        gGearGradient4.render(
                            RelativePosition1Object(SCREEN_WIDTH,	gGearGradient4.getWidth(),	RELATIVE_PLACE_RPM_X),
                            RelativePosition1Object(SCREEN_HEIGHT,	gGearGradient4.getHeight(),	RELATIVE_PLACE_RPM_Y),
                            gRenderer );
                        break;
                    case 5:
                        gGearGradient5.render(
                            RelativePosition1Object(SCREEN_WIDTH,	gGearGradient5.getWidth(),	RELATIVE_PLACE_RPM_X),
                            RelativePosition1Object(SCREEN_HEIGHT,	gGearGradient5.getHeight(),	RELATIVE_PLACE_RPM_Y),
                            gRenderer );
                        break;
                    case 6:
                        gGearGradient6.render(
                            RelativePosition1Object(SCREEN_WIDTH,	gGearGradient6.getWidth(),	RELATIVE_PLACE_RPM_X),
                            RelativePosition1Object(SCREEN_HEIGHT,	gGearGradient6.getHeight(),	RELATIVE_PLACE_RPM_Y),
                            gRenderer );
                        break;

                }

				reloadText();
				gTextGear.render(
                        ( SCREEN_WIDTH - gTextGear.getWidth() )*RELATIVE_PLACE_FONT_GEAR_X,
                        ( SCREEN_HEIGHT - gTextGear.getHeight() )*RELATIVE_PLACE_FONT_GEAR_Y,
                        gRenderer);
				gTextVelocity.render(
                        ( SCREEN_WIDTH - gTextVelocity.getWidth() )*RELATIVE_PLACE_FONT_VELOCITY_X,
                        ( SCREEN_HEIGHT - gTextVelocity.getHeight() )*RELATIVE_PLACE_FONT_VELOCITY_Y,
                        gRenderer);

				//load the polygon:
				//Try to rotate according to Yaw
				//rotatePts(Xtrack, Ytrack, n, 0.25 , 200,200 ,Xtrack_Updated ,Ytrack_Updated);
				//shiftPTS(Xtrack,Ytrack,n,Xtrack[(degrees/10+1)%n]-Xtrack[degrees/10%n],Ytrack[(degrees/10+1)%n]-Ytrack[degrees/10%n],Xtrack_Updated, 	Ytrack_Updated);


				//if (polygonRGBA(gRenderer,Xtrack_Updated, Ytrack_Updated,n,255, 255, 255, 155)
//                Point p(300,300);
//               utils.rotateVec(pts , Updated_pts , p , 20);
		//Point delXY(-100,-100);
		//utils.translateVec(pts ,Updated_pts ,delXY);
		//utils.strechVec(pts ,Updated_pts, p ,0.5, 'y');
		//utils.strechVec(pts ,Updated_pts, p ,0.2, 'x');
				for(ptsIndex = 1; ptsIndex <= pts.size() ;ptsIndex++)
				{
					lineRGBA(gRenderer ,Updated_pts[ptsIndex-1].X ,Updated_pts[ptsIndex-1].Y ,Updated_pts[ptsIndex % pts.size()].X ,Updated_pts[ptsIndex % pts.size()].Y ,100,100,255,155);

//				for(ptsIndex = 1; ptsIndex <= pts.size() ;ptsIndex++)
//				{
//					lineRGBA(gRenderer ,pts[ptsIndex-1].X ,pts[ptsIndex-1].Y ,pts[ptsIndex % pts.size()].X ,pts[ptsIndex % pts.size()].Y ,100,100,255,155);
//					//ptsUpdated.push_back(pts[ptsIndex]);
//				}
//                Point p(300,300);
//               utils.rotateVec(pts , Updated_pts , p , 20);
//				for(ptsIndex = 1; ptsIndex <= pts.size() ;ptsIndex++)
//				{
//					lineRGBA(gRenderer ,Updated_pts[ptsIndex-1].X ,Updated_pts[ptsIndex-1].Y ,Updated_pts[ptsIndex % pts.size()].X ,Updated_pts[ptsIndex % pts.size()].Y ,100,100,255,155);
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

