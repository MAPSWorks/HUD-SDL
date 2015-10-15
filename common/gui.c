/*This source code copyrighted by Lazy Foo' Productions (2004-2015)
  and may not be redistributed without written permission.*/

///Using SDL, SDL_image, SDL_ttf, standard IO, math, and strings
#include "gui.h"
#include "sensorenv.h"
#include <time.h>
#include <iostream>
#include <vector>
#include "common.h"
#include "bluetooth_top.h"

//#define TRACK_FPS 1

extern BT_data bt_data;
extern VnDeviceCompositeData sensorData;
extern bool globQuitSig;
int velocity = 0;
int gear = 0;
int RPM = 0;



///Eden's Block

Point scrP(0,0);
Point scrOrigin(SCREEN_WIDTH/2,SCREEN_HEIGHT/2);

VnVector3 sensorVel;
double newLat=32.0;
double newLon=35.0;
double newAlt = 0;
bool frameDef = false;
std::vector<double> frame;

bool flagSim = true;
VnVector3 vnZero = {0};

std::vector<Point> originalPts;
std::vector<Point> mapPts;
std::vector<VnVector3> vecVelocity;
std::vector<double> vecLatitude;
std::vector<double> vecLongitude;
std::vector<double> vecAltitude;

std::vector<Point> originalPts_Prev;
std::vector<Point> mapPts_Prev;
std::vector<VnVector3> vecVelocity_Prev;
std::vector<double> vecLatitude_Prev;
std::vector<double> vecLongitude_Prev;
std::vector<double> vecAltitude_Prev;
int LapCounter=0;
double LapTime=0;
int trailPointIdx = -1;
Point deltaXY(0,0);
Point origin(0,0);
bool newPointSampled = false;
bool gpsSig = false;
int counter = 0; //for simulation
std::vector<double> quaternion;
std::vector<double> axis;
double scrRotAng = 0;
double angRot = 0;
Coordinate prevCo(0,0);
Coordinate currCo(0,0);
Coordinate dist1(0,0);
Coordinate dist2(0,0);
double numFrames;

double yaw;
double pitch;
double roll;
std::vector<Point> scrPts;
///

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
LTexture gCarCrossTexture;
LTexture gGPSSignalTexture;
LTexture gCarMapMarkTexture;
LTexture gBlueToothTexture;
LTexture gpsNOSIgnalTexture;

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
		gCarCrossTexture.loadFromFile( PROJ_HOME "/resources/cross.png" ) &&
		gCarArrowTexture.loadFromFile( PROJ_HOME "/resources/arrow1.png" ) &&
		gCarMapMarkTexture.loadFromFile( PROJ_HOME "/resources/map_marker.png" ) &&
		gBlueToothTexture.loadFromFile( PROJ_HOME "/resources/btLogo2.png" ) &&
		gpsNOSIgnalTexture.loadFromFile( PROJ_HOME "/resources/gpsNOSIgnal.png" ) &&
		gGPSSignalTexture.loadFromFile( PROJ_HOME "/resources/gpstracksicon.png" );
}


bool reloadText()
{
	std::string strVelocity = std::to_string(velocity);
	std::string strGear = std::to_string(gear);

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
	gCarCrossTexture.free();
	gGPSSignalTexture.free();
	gCarMapMarkTexture.free();
	gBlueToothTexture.free();
	gpsNOSIgnalTexture.free();

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
	//int numFrames = 0;

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
                //printf("MA filter - Start\n");
                newLon = 0;
                newLat = 0;
                newAlt = 0;
                pitch = 0;
                yaw = 0;
                roll = 0;
                sensorVel = {0};
                for(unsigned int j = 0;j<LEN_FILTER; ++j){
                    sensorVel.c0 += sensorData.velBody.c0;
                    sensorVel.c1 += sensorData.velBody.c1;
                    sensorVel.c2 += sensorData.velBody.c2;
                    newLat += sensorData.latitudeLongitudeAltitude.c0;
                    newLon += sensorData.latitudeLongitudeAltitude.c1;
                    newAlt += sensorData.latitudeLongitudeAltitude.c2;
                    yaw += sensorData.ypr.yaw;
                    roll += sensorData.ypr.roll;
                    pitch += sensorData.ypr.pitch;
                }
                newLat = newLat/LEN_FILTER;
                newLon = newLon/LEN_FILTER;
                newAlt = newAlt/LEN_FILTER;
                yaw = yaw/LEN_FILTER;
                pitch = pitch/LEN_FILTER;
                roll = roll/LEN_FILTER;
                sensorVel.c0 = sensorVel.c0/LEN_FILTER;
                sensorVel.c1 = sensorVel.c1/LEN_FILTER;
                sensorVel.c2 = sensorVel.c2/LEN_FILTER;
                if(newLat == 0 || newLon == 0)
                {
                    sensorVel = {0};
                }

                //printf("MA filter - Cleared\n");
                if(newLat!=0){
                printf("newLat = %f newLon = %f newAlt = %f\n",newLat,newLon,newAlt);
                }
                ****End filter*****/
                //utils.benchTest(vecLatitude ,vecLongitude ,newLat ,newLon,sensorVel,counter);


                /**** Simulate GPS for debug****/
                yaw = sensorData.ypr.yaw;
                roll = sensorData.ypr.roll;
                pitch = sensorData.ypr.pitch;


                dist1.X=0;
                dist1.Y=0;
                dist2.Y=0;
                counter++;
                newAlt = 0;
                if(counter<=100){
                    newLat = 35+0.001*cos(PI/50*counter);
                    newLon = 32+0.001*sin(PI/50*counter);
                    if(vecLatitude.size()>1){
                    utils.gps2linDist(dist1,vecLatitude[vecLatitude.size()-1],vecLongitude[vecLatitude.size()-1]);
                    utils.gps2linDist(dist2,vecLatitude[vecLatitude.size()-2],vecLongitude[vecLatitude.size()-2]);
                    sensorVel.c0 = dist1.X-dist2.X;
                    sensorVel.c1 = dist1.Y-dist2.Y;
                    }
                    else
                    {
                    sensorVel.c0 = 0;
                    sensorVel.c1 = 0;
                    }
                    sensorVel.c2 = 0;
                    //printf("1newLat,newLon = (%f,%f)\n",newLat,newLon);
                }
                else if(counter<=200){
                    newLat = 35 + 0.001*cos(PI-PI/50*counter)+0.002;
                    newLon = 32 + 0.001*sin(PI-PI/50*counter);
                    if(vecLatitude.size()>1){
                    utils.gps2linDist(dist1,vecLatitude[vecLatitude.size()-1],vecLongitude[vecLatitude.size()-1]);
                    utils.gps2linDist(dist2,vecLatitude[vecLatitude.size()-2],vecLongitude[vecLatitude.size()-2]);
                    sensorVel.c0 = dist1.X-dist2.X;
                    sensorVel.c1 = dist1.Y-dist2.Y;
                    }
                    else
                    {
                    sensorVel.c0 = 0;
                    sensorVel.c1 = 0;
                    }
                    sensorVel.c2 = 0;
                    //printf("2newLat,newLon = (%f,%f)\n",newLat,newLon);
                }
                if(counter == 200)
                    counter = 0;



#ifdef TRACK_FPS
				fps = ( numFrames/(float)(SDL_GetTicks() - startTime) )*1000;
				printf("FPS: %lf\n", fps);
#endif
				horDeg = 90 + (double)sensorData.ypr.roll;
				RPMint = (int)bt_data.rpm;
				velocityInt = (int)bt_data.velo;
				velocity = velocityInt;
				degrees     = (velocityInt*MAX_DEGREE)/MAX_VELOCITY + DEGREES_OFFSET;
				gearInt = bt_data.gear;
				gear = gearInt;
				strGear = std::to_string(gearInt);

				//Clear screen
				SDL_SetRenderDrawColor( gRenderer, 0, 0, 0, 0 );	//	background screen color
				SDL_RenderClear( gRenderer );

				//Render current frame
				//pngs
				gGearGradient[(RPMint*6)/MAX_RPM].renderRelToScrn(RELATIVE_PLACE_RPM_X, RELATIVE_PLACE_RPM_Y );
				gArtHorzTexture.renderRelToScrn(RELATIVE_PLACE_ARTHORZ_X - 0.002 + 0.002, RELATIVE_PLACE_ARTHORZ_Y + 0.001 + 0.003 , 0.0 - horDeg);
				gCarCrossTexture.renderRelToScrn(RELATIVE_PLACE_ARTHORZ_X + 0.002, RELATIVE_PLACE_ARTHORZ_Y + 0.003 , 0.0);
				gVelocityGradient.renderRelToScrn(RELATIVE_PLACE_VELOCITY_G_X, RELATIVE_PLACE_VELOCITY_G_Y);
				gNeedleTexture.renderRelToScrnRel2Object(RELATIVE_PLACE_SPEEDOMETER_X, RELATIVE_PLACE_SPEEDOMETER_Y, gVelocityGradient, -50.0+degrees);
				gCarArrowTexture.renderRelToScrn(RELATIVE_PLACE_ARROW_X + 0.487,RELATIVE_PLACE_ARROW_Y - 0.985);
				gCarMapMarkTexture.renderRelToScrn(RELATIVE_PLACE_ARROW_X + 0.482,RELATIVE_PLACE_ARROW_Y - 1.045);
				if(newLat == 0 && newLon == 0 && newAlt == 0)
                    gpsNOSIgnalTexture.renderRelToScrn(0.9,0.05);
                else
                    gGPSSignalTexture.renderRelToScrn(0.9,0.05);
				gBlueToothTexture.renderRelToScrn(0.965,0.065);

				//txts
				reloadText();
				gTextGear.renderTXTRelToScrn(RELATIVE_PLACE_FONT_GEAR_X, RELATIVE_PLACE_FONT_GEAR_Y);
				gTextVelocity.renderTXTRelToScrn(RELATIVE_PLACE_FONT_VELOCITY_X, RELATIVE_PLACE_FONT_VELOCITY_Y);

/********************Eden's block************************/
                //printf("test1\n");
                newPointSampled = utils.buildMap(vecVelocity,vecLatitude ,vecLongitude, vecAltitude, newLat, newLon ,newAlt ,originalPts, sensorVel,frameDef,frame);
                //utils.UpdateMap(originalPts,originalPts_Old,mapPts,mapPts_Old);
                //if(!gpsSig){continue;}
                //printf("test2\n");
                trailPointIdx = utils.isClosedLoop(vecVelocity,vecLatitude,vecLongitude);
                //This is where the loop starts (Also where it ends)
                //printf("test3\n");
                if(trailPointIdx>-1)
                {
                    //printf("trailPointIndx = %d\n",trailPointIdx);
                    //printf("test4\n");
                    frameDef = true;//Very questionalble!!!!!!!
                    printf("closedLoopDetected\n");
                    vecVelocity_Prev.clear();
                    vecLatitude_Prev.clear();
                    vecAltitude_Prev.clear();
                    vecLongitude_Prev.clear();
                    originalPts_Prev.clear();
                    mapPts_Prev.clear();

                    ///////////////////////////////////////////////// TODO: move to a decent location
                    /*
                    SDL_RWops *latlogfile = SDL_RWFromFile(PROJ_HOME "/misc/data_logs/lat.log", "w");
                    SDL_RWops *lonlogfile = SDL_RWFromFile(PROJ_HOME "/misc/data_logs/lon.log", "w");
                    char str[BUF_SIZE];
                    const char* del = "#########\n";
                    SDL_RWwrite(latlogfile, del, sizeof(char), SDL_strlen(del));
                    SDL_RWwrite(lonlogfile, del, sizeof(char), SDL_strlen(del));
                    for(auto it = vecLatitude.begin(); it != vecLatitude.end(); ++it) {
                        sprintf(str,"%lf\n", *it);
                        SDL_RWwrite(latlogfile, str, sizeof(char), SDL_strlen(str));
                    }
                    for(auto it = vecLongitude.begin(); it != vecLongitude.end(); ++it) {
                        sprintf(str,"%lf\n", *it);
                        SDL_RWwrite(lonlogfile, str, sizeof(char), SDL_strlen(str));
                    }
                    SDL_RWclose(latlogfile);
                    SDL_RWclose(lonlogfile);
                    /////////////////////////////////////////////////
                    */
                    //printf("test1\n");
                    //printf("test5\n");
                    for(unsigned int i=0/*trailPointIdx*/ ; i<vecLatitude.size() ; ++i)
                    {
                        vecVelocity_Prev.push_back(vecVelocity[i]);
                        vecLatitude_Prev.push_back(vecLatitude[i]);
                        vecLongitude_Prev.push_back(vecLongitude[i]);
                        vecAltitude_Prev.push_back(vecAltitude[i]);
                        originalPts_Prev.push_back(originalPts[i]);
                    }
                    //printf("test2\n");
                    vecVelocity_Prev.push_back(vecVelocity[trailPointIdx]);
                    vecLatitude_Prev.push_back(vecLatitude[trailPointIdx]);
                    vecLongitude_Prev.push_back(vecLongitude[trailPointIdx]);
                    vecAltitude_Prev.push_back(vecAltitude[trailPointIdx]);
                    originalPts_Prev.push_back(originalPts[trailPointIdx]);
                    vecVelocity.clear();
                    vecLatitude.clear();
                    vecLongitude.clear();
                    vecAltitude.clear();
                    originalPts.clear();
                    mapPts.clear();
                }
                //printf("test3\n");
                //origin.X=ARROW_POS_X;
                //origin.Y=ARROW_POS_Y;
                if(newPointSampled){
                    origin.X = originalPts[originalPts.size()-1].X ;
                    origin.Y = originalPts[originalPts.size()-1].Y ;
                    deltaXY.X = ARROW_POS_X - originalPts[originalPts.size()-1].X ;
                    deltaXY.Y = ARROW_POS_Y - originalPts[originalPts.size()-1].Y ;
                    utils.gps2linDist(prevCo,vecLatitude[vecLatitude.size()-1],vecLongitude[vecLongitude.size()-1]);
                    utils.gps2linDist(currCo,vecLatitude[vecLatitude.size()-2],vecLongitude[vecLongitude.size()-2]);

                    if(utils.gpsSinal(currCo) && utils.gpsSinal(prevCo) && vecLatitude.size()>1)
                        angRot = utils.CoordinateAngle(prevCo,currCo);
                    else
                        angRot = 0;
                    //printf("prevCo=(%f,%f),currCo = (%f,%f)\n",prevCo.X,prevCo.Y,currCo.X,currCo.Y);
                    //printf("angleRot = %f\n",angRot);
                    //Updating map stuff
                    //printf("test5\n");
                    utils.UpdateMap(originalPts,mapPts,vecVelocity,origin, deltaXY,newPointSampled,angRot);
                    //printf("test6\n");
                    utils.gps2frame(vecLatitude_Prev,vecLongitude_Prev,originalPts_Prev,frameDef,frame);
                    //printf("test7\n");
                    utils.UpdateMap(originalPts_Prev,mapPts_Prev,vecVelocity,origin, deltaXY,newPointSampled,angRot);
                    //printf("tes   t8\n");
                }
                //printf("test10\n");
                //End here updating

                /***Trivial update for debug**/
                //mapPts = originalPts;
                //utils.gps2frame(vecLatitude_Prev,vecLongitude_Prev,originalPts_Prev,frameDef,frame);
                //mapPts_Prev = originalPts_Prev;
                //printf("sizeOfmapPrev = %d\n",mapPts_Prev.size());
                //printf("sizeOfmap = %d\n",mapPts.size());
                /*******************/
                //printf("test10\n");
                if(vecAltitude.size()>1){
                	for(unsigned int idx = 1; idx < mapPts_Prev.size() ; ++idx)
                	{
                	    thickLineRGBA(gRenderer ,mapPts_Prev[idx-1].X ,mapPts_Prev[idx-1].Y ,mapPts_Prev[idx].X ,mapPts_Prev[idx].Y,LINE_THICKNESS ,50,100,255,100);
                	}
                	//Draw new Map green
			for(unsigned int idx = 1; idx < mapPts.size() ; ++idx)
			{
				thickLineRGBA(gRenderer ,mapPts[idx-1].X ,mapPts[idx-1].Y ,mapPts[idx].X ,mapPts[idx].Y,LINE_THICKNESS ,50,200,255,255);
			}
		}
		//scrP.Y = SCREEN_HEIGHT/2 + EYE_RELEIF/PHYSICAL_PIXEL_SIZE*tan(PI/180*pitch);
		//scrP.X = SCREEN_WIDTH/2 - EYE_RELEIF/PHYSICAL_PIXEL_SIZE*tan(PI/180*yaw);
		//utils.rotatePoint(scrP,scrP,scrOrigin,roll);
		//thickLineRGBA(gRenderer ,scrP.X,scrP.Y,scrP.X,scrP.Y,LINE_THICKNESS , 255,255,255,255);

        //printf("ypr = %f %f %f\n" ,yaw,pitch,roll);
	//			if(vecAltitude_Prev.size()>0 && vecAltitude.size()>0){
         //           utils.renderTrail2scr(vecLatitude[vecLatitude.size()-1],vecLongitude[vecLongitude.size()-1],vecAltitude[vecAltitude.size()-1] ,vecLatitude_Prev,vecLongitude_Prev,vecAltitude_Prev,yaw, pitch, roll, scrPts);
          //          for(unsigned int idx = 1; idx<scrPts.size() ; ++idx)
           //         {
            //            thickLineRGBA(gRenderer ,scrPts[idx-1].X ,scrPts[idx-1].Y ,scrPts[idx].X ,scrPts[idx].Y,LINE_THICKNESS ,50,100,100,155);
             //           printf("scrPts = (%d,%d) \n",scrPts[idx-1].X,scrPts[idx-1].Y);
              //      }
               // }
                //
                //thickLineRGBA(gRenderer ,1280/2, 720/2 ,1280/2 ,720/2,LINE_THICKNESS ,255,255,255,155);
				//printf("test12\n");
				//printf("ypr = (%f,%f,%f)\n",sensorData.ypr.yaw,sensorData.ypr.pitch,sensorData.ypr.roll);
				//printf("MF!!!!!");
                //printf("Q = (%f,%f,%f,%f)\n",sensorData.quaternion.w,sensorData.quaternion.x,sensorData.quaternion.y,sensorData.quaternion.z);
                //if(vecLatitude.size()>0)
                  //  utils.renderTrail2scr(vecLatitude[vecLatitude.size()-1],vecLongitude[vecLongitude.size()-1],vecAltitude[vecAltitude.size()-1] ,vecLatitude_Prev,vecLongitude_Prev,vecAltitude_Prev,sensorData.ypr.yaw, sensorData.ypr.pitch, sensorData.ypr.roll,scrPts);


                utils.coordinate2Scr(32.0 ,35.0 ,0.0 ,32.001 ,35.001 ,0.0 ,pitch ,yaw ,roll ,scrP);
                thickLineRGBA(gRenderer ,scrP.X ,scrP.Y ,scrP.X ,scrP.Y, 2*LINE_THICKNESS ,50,200,255,255);

/*************************************************************************/
				SDL_RenderPresent( gRenderer );
			}
		}
	}

	//Free resources and close SDL
	close();

	return 0;
}

