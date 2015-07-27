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


/***Eden's Block**/


VnVector3 sensorVel;
double newLat=32.0;
double newLon=35.0;



std::vector<Point> originalPts;
std::vector<Point> mapPts;
std::vector<VnVector3> vecVelocity;
std::vector<double> vecLatitude;
std::vector<double> vecLongitude;

std::vector<Point> originalPts_Old;
std::vector<Point> mapPts_Old;
std::vector<VnVector3> vecVelocity_Old;
std::vector<double> vecLatitude_Old;
std::vector<double> vecLongitude_Old;
int LapCounter=0;
double LapTime=0;
int trailPointIdx = -1;

/**********/

//The window we'll be rendering to
SDL_Window* gWindow = NULL;

//The window renderer
SDL_Renderer* gRenderer = NULL;
SDL_Renderer** globgRenderer = &gRenderer;

//Globally used font
TTF_Font *gFont = NULL, *gDigitalFont = NULL;

//text for the velocity and gear.
LTexture gTextVelocity, gTextGear;
LTexture gTextGPS;

//creating the gear and the velocity gradients.
LTexture gVelocityGradient;
LTexture gGearGradient[6];
LTexture gNeedleTexture;
LTexture gCarArrowTexture;


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
		gNeedleTexture.loadFromFile( PROJ_HOME "/resources/needle-fioptics2_trial.png" ) 	&&
		gArtHorzTexture.loadFromFile( PROJ_HOME "/resources/artHorz.png" ) 		&&
		gTextVelocity.loadFromRenderedText( "Tadaa", VEL_FONT_COLOR,gDigitalFont )	&&
		gTextGear.loadFromRenderedText( "Tadaa", GEAR_FONT_COLOR, gDigitalFont ) &&
		gCarArrowTexture.loadFromFile( PROJ_HOME "/resources/arrow.png" );
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
	gCarArrowTexture.free();

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
	int gearInt = 1;
	int RPMint = 0;


	std::string strVelocity = std::to_string(velocityInt);
	std::string strGear = std::to_string(gearInt);


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

                /******MA Filter*****
                newLon = 0;
                newLat = 0;
                sensorVel = {0};
                for(unsigned int j = 0;j<LEN_FILTER; ++j){
                    sensorVel.c0 += sensorData.velBody.c0;
                    sensorVel.c1 += sensorData.velBody.c1;
                    sensorVel.c2 += sensorData.velBody.c2;
                    newLat += sensorData.latitudeLongitudeAltitude.c0;
                    newLon += sensorData.latitudeLongitudeAltitude.c1;
                    }
                newLat = newLat/LEN_FILTER;
                newLon = newLon/LEN_FILTER;
                //printf("newLat = %f newLon = %f\n",newLat,newLon);
                ****End filter*****/

                /**** Simulate GPS for debug****/
                if(vecLatitude.size()<=10){
                    newLat+=0.00001;
                    newLon+=0.00002;
                    sensorVel.c0 = 10;
                    sensorVel.c1 = 20;
                    sensorVel.c2 = 0;
                    }
                else if(vecLatitude.size()<=20){
                    newLat+=0.00003;
                    newLon+=0.00001;
                    sensorVel.c0 = 30;
                    sensorVel.c1 = 10;
                    sensorVel.c2 = 0;
                }
                else if(vecLatitude.size()<=30){
                    newLat+=0.00004;
                    sensorVel.c0 = 40;
                    sensorVel.c1 = 0;
                    sensorVel.c2 = 0;
                }
                else if(vecLatitude.size()<=40){
                    newLat-=0.00001;
                    newLon-=0.00002;
                    sensorVel.c0 = -10;
                    sensorVel.c1 = -20;
                    sensorVel.c2 = 0;
                }
                else if(vecLatitude.size()<=50){
                    newLat-=0.00003;
                    newLon-=0.00001;
                    sensorVel.c0 = -30;
                    sensorVel.c1 = -10;
                    sensorVel.c2 = 0;
                }
                else if(vecLatitude.size()<=60){
                    newLat-=0.00004;
                    sensorVel.c0 = -40;
                    sensorVel.c1 = 0;
                    sensorVel.c2 = 0;
                }
                else if(vecLatitude.size()<=70){
                    newLat+=0.00001;
                    newLon+=0.00002;
                    sensorVel.c0 = 10;
                    sensorVel.c1 = 20;
                    sensorVel.c2 = 0;
                    }
                else if(vecLatitude.size()<=80){
                    newLat+=0.00003;
                    newLon+=0.00001;
                    sensorVel.c0 = 30;
                    sensorVel.c1 = 10;
                    sensorVel.c2 = 0;
                }
                else if(vecLatitude.size()<=90){
                    newLat+=0.00004;
                    sensorVel.c0 = 40;
                    sensorVel.c1 = 0;
                    sensorVel.c2 = 0;
                }
                else if(vecLatitude.size()<=100){
                    newLat-=0.00001;
                    newLon-=0.00002;
                    sensorVel.c0 = -10;
                    sensorVel.c1 = -20;
                    sensorVel.c2 = 0;
                }
                else if(vecLatitude.size()<=110){
                    newLat-=0.00003;
                    newLon-=0.00001;
                    sensorVel.c0 = -30;
                    sensorVel.c1 = -10;
                    sensorVel.c2 = 0;
                }
                else if(vecLatitude.size()<=120){
                    newLat-=0.00004;
                    sensorVel.c0 = -40;
                    sensorVel.c1 = 0;
                    sensorVel.c2 = 0;
                }
                //printf("newLat = %f , newLon = %f\n",newLat , newLon);
                /*******************************/

                //printf("Lat = %f , Lon = %f",newLat,newLon);

				// for demo only. to be connected to the actual values
				/*numFrames++;
				velocityInt += VELOCITY_STEP;
				RPMint      += RPM_STEP;
				if(numFrames%10 == 0){
				gearInt    += GEAR_STEP;
				}
				velocityInt %= MAX_VELOCITY;
				RPMint      %= MAX_RPM;
				gearInt     %= MAX_GEAR;
				if(gearInt==0) gearInt=1;
				degrees     = (velocityInt*MAX_DEGREE)/MAX_VELOCITY + DEGREES_OFFSET;

				// connecting with the global variables
				RPM = RPMint;
				velocity = velocityInt;
				gear = gearInt;
				*/

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
				gCarArrowTexture.renderRelToScrn(RELATIVE_PLACE_ARROW_X,RELATIVE_PLACE_ARROW_Y);

				//txts
				reloadText();
				gTextGear.renderTXTRelToScrn(RELATIVE_PLACE_FONT_GEAR_X, RELATIVE_PLACE_FONT_GEAR_Y);
				gTextVelocity.renderTXTRelToScrn(RELATIVE_PLACE_FONT_VELOCITY_X, RELATIVE_PLACE_FONT_VELOCITY_Y);

/********************Eden's MF block bitch!!************************/

                utils.buildMap(vecVelocity,vecLatitude ,vecLongitude, newLat, newLon ,originalPts, sensorVel);
                //utils.UpdateMap(originalPts,originalPts_Old,mapPts,mapPts_Old);
                trailPointIdx = utils.isClosedLoop(vecVelocity,vecLatitude,vecLongitude);
                //This is where the loop starts (Also where it ends)
                if(trailPointIdx>-1)
                {
                    printf("closedLoopDetected\n");
                    vecVelocity_Old.clear();
                    vecLatitude_Old.clear();
                    vecLongitude_Old.clear();
                    originalPts_Old.clear();
                    mapPts_Old.clear();
                    for(unsigned int i=0 ; i<vecLatitude.size() ; ++i)
                    {
                        vecVelocity_Old.push_back(vecVelocity[i]);
                        vecLatitude_Old.push_back(vecLatitude[i]);
                        vecLongitude_Old.push_back(vecLongitude[i]);
                        originalPts_Old.push_back(originalPts[i]);
                    }
                    vecVelocity.clear();
                    vecLatitude.clear();
                    vecLongitude.clear();
                    originalPts.clear();
                    mapPts.clear();
                    vecVelocity.push_back(vecVelocity_Old[trailPointIdx]);
                    vecLatitude.push_back(vecLatitude_Old[trailPointIdx]);
                    vecLongitude.push_back(vecLongitude_Old[trailPointIdx]);
                    originalPts.push_back(originalPts[trailPointIdx]);
                }
                mapPts = originalPts;
                mapPts_Old=originalPts_Old;
                for(unsigned int idx = 1; idx < originalPts_Old.size() ; ++idx)
                {
                    thickLineRGBA(gRenderer ,mapPts_Old[idx-1].X ,mapPts_Old[idx-1].Y ,mapPts_Old[idx].X ,mapPts_Old[idx].Y,LINE_THICKNESS ,100,100,150,155);
                }
                //Draw new Map green
				for(unsigned int idx = 1; idx < originalPts.size() ; ++idx)
				{
					thickLineRGBA(gRenderer ,mapPts[idx-1].X ,mapPts[idx-1].Y ,mapPts[idx].X ,mapPts[idx].Y,LINE_THICKNESS ,50,255,50,155);
				}

/*************************************************************************/
				SDL_RenderPresent( gRenderer );
			}
		}
	}

	//Free resources and close SDL
	close();

	return 0;
}

