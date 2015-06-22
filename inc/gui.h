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

//GPS and MAP Stuff
#define Earth_Radius 6372000.797560856 //Meters
#define PI 3.14159265359
#define MAP_FRAME_WIDTH 400
#define MAP_FRAME_LENGTH 400
#define MAP_FRAME_POS_X 300
#define MAP_FRAME_POS_Y 300

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
	void rotatePoint(Point& point ,Point& updated_point, Point& origin ,double ang_Deg);
	void strech(Point& point ,Point& updated_point , Point& origin ,double factor, char xy);
	void gps2linDist(Point& updated_point ,double lat, double lon);
	void rotateVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts, Point& origin ,double ang_Deg);
	void strechVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts, Point& origin ,double factor, char xy);
	void normVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts);
	void translateVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts ,Point& deltaXY);
	void GPS2ImageFrame(std::vector<Point>& pts ,double lat, double lon);
	bool inNeighbourhood(Point& p1, Point& p2, double radius);
	double velocityAng(std::vector<double>& velocity);
	void samplePoints(std::vector<double>& vecLatitude,std::vector<double>& vecLongitude,double newLat,double newLon);
};

void* gui_main(void* arg);


#endif
