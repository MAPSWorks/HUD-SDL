#ifndef GUI_H_
#define GUI_H_

#include "LTexture.h"
#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <SDL2_gfxPrimitives.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include "common.h"
#include <vector>
#include <iostream>

//Screen dimension constants
#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 720

#define MAX_RPM         12000
#define MAX_VELOCITY    180
#define MAX_DEGREE      130
#define MAX_GEAR        5
#define DEGREES_OFFSET  90

// for the wassach Biba is gay :)
#define VELOCITY_STEP   2
#define RPM_STEP        50
#define GEAR_STEP       1
#define FONT_7_SIZE     150

// velocity background
#define RELATIVE_PLACE_VELOCITY_G_X 5.34/10
#define RELATIVE_PLACE_VELOCITY_G_Y 8.74/10

#define RELATIVE_PLACE_RPM_X 5.1/10
#define RELATIVE_PLACE_RPM_Y 8.7/10

// fraction that will represent the relative place of the speedometer relative in the screen
#define RELATIVE_PLACE_SPEEDOMETER_X 6.0/10 // we must use .0 so it will considered as double.
#define RELATIVE_PLACE_SPEEDOMETER_Y 9.5/10

//Artificial Horizon
#define RELATIVE_PLACE_ARTHORZ_X 5.0/10
#define RELATIVE_PLACE_ARTHORZ_Y 5.0/10

#define VEL_FONT_COLOR 	{ 0xFF, 0xFF, 0xFF}
#define GEAR_FONT_COLOR { 0xFF, 0xFF, 0xFF}

//velocity 7 seg font place
#define RELATIVE_PLACE_FONT_VELOCITY_X 5.7/10
#define RELATIVE_PLACE_FONT_VELOCITY_Y 9.0/10

//Gear 7 segment font place
#define RELATIVE_PLACE_FONT_GEAR_X 9.0/10
#define RELATIVE_PLACE_FONT_GEAR_Y 8.0/10

//GPS and MAP Stuff
#define Earth_Radius 6372000.797560856 //Meters
#define PI 3.14159265359
#define MAP_FRAME_WIDTH 400
#define MAP_FRAME_LENGTH 400
#define MAP_FRAME_POS_X 300
#define MAP_FRAME_POS_Y 300

//Map Stuff
#define LINE_THICKNESS 2 //for map
#define RELATIVE_PLACE_ARROW_X -1.0/10
#define RELATIVE_PLACE_ARROW_Y 8.0/10


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


class Point{
public:
	short X;
	short Y;
	Point(int X, int Y);
};
class guiUtils{
public:
	//Delay time
	void waitFor (unsigned int secs);
  //Point functions
	void rotatePoint(Point& point ,Point& updated_point, Point& origin ,double ang_Deg);
	void strech(Point& point ,Point& updated_point , Point& origin ,double factor, char xy);
	void gps2linDist(Point& updated_point ,double lat, double lon);
	//Vec functions
	void rotateVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts, Point& origin ,double ang_Deg);
	void strechVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts, Point& origin ,double factor, char xy);
	void translateVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts ,Point& deltaXY);
	//General
	double movingAveragefilter(std::vector<double>& vectorIN,double newTerm,unsigned int num_terms);
	bool inNeighbourhood(Point& p1, Point& p2, double radius);
	double velocityAng(std::vector<double>& velocity);
	//Point sampling
	bool sampleNewPoint(std::vector<double>& vecLatitude,std::vector<double>& vecLongitude,double newLat,double newLon, unsigned int& numLastPointsInRadius);
	//Map utils
	void normVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts);
  void gps2frame(std::vector<double>& vecLatitude,std::vector<double>& vecLongitude,std::vector<Point>& FramePts);
  void zoomMap(std::vector<Point>& pts ,std::vector<Point>& Updated_pts,double factor,Point origin);
  void setOrientation(std::vector<Point> originalPts, std::vector<Point> mapPts , std::vector<double> velocity);
  void setPosition(std::vector<Point> mapPts , Point prev_location,Point next_location);
  void setScale(std::vector<Point> originalPts, std::vector<Point> mapPts , std::vector<double> velocity);
  //High level
  void buildMap(std::vector<double>& vecLatitude ,std::vector<double>& vecLongitude, double newLat, double newLon ,std::vector<Point> originalPts ,std::vector<Point> mapPts,unsigned int& numLastPointsInRadius,std::vector<double> velocity);
};

void* gui_main(void* arg);


#endif
