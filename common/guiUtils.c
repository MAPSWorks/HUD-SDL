/***** Utilities for gui ****/



//Local defines
#define SAMPLE_RADIUS 3
#define FILTER_WIDTH 10
#define VEHICLE_ORIGIN_X 0
#define VEHICLE_ORIGIN_Y 0

//Local includes
#include "gui.h"
#include <iostream>
#include <vector>
#include <math.h>
#include "sensorenv.h"

//General
double guiUtils::movingAveragefilter(std::vector<double>& vectorIN,double newTerm,unsigned int num_terms)
{
	if(vectorIN.size()==0) return -1;
	double some = 0;
	unsigned int actural_num_terms = (newTerm < vectorIN.size()) ? newTerm:vectorIN.size();
	for(unsigned int i=0 ; i<actural_num_terms ; i++)
	{
		some += vectorIN[i];
	}
	return some/actural_num_terms;
}


Point::Point(int X, int Y) : X(X), Y(Y) {};


//SUB UTILITIES
//point functions
void guiUtils::rotatePoint(Point& point ,Point& updated_point, Point& origin ,double ang_Deg)
{
	updated_point.X = ( point.X - origin.X )*cos(ang_Deg*PI/180) + ( point.Y - origin.Y )*sin(ang_Deg*PI/180) + origin.X;
	updated_point.Y = -( point.X - origin.X )*sin(ang_Deg*PI/180) + ( point.Y - origin.Y )*cos(ang_Deg*PI/180) + origin.Y;
}

void guiUtils::strech(Point& point ,Point& updated_point , Point& origin ,double factor, char xy)
{
	if(xy == 'x') updated_point.X = ( point.X - origin.X )*factor + origin.X;

	else if(xy == 'y') updated_point.X = ( point.Y - origin.Y )*factor + origin.Y;

	else{
		 std::cout << "error in method strech input must be either x or y";
	}
}

void guiUtils::gps2linDist(Point& updated_point ,double lat, double lon)
{
	updated_point.X = Earth_Radius*cos(lat)*cos(lon);
	updated_point.Y = Earth_Radius*cos(lat)*sin(lon);
}




//vector functions
void guiUtils::rotateVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts, Point& origin ,double ang_Deg)
{
	for(unsigned int i = 0 ;  i<pts.size() ; i++)
	{
		rotatePoint(pts[i] ,Updated_pts[i], origin ,ang_Deg);
	}
}

void guiUtils::strechVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts, Point& origin ,double factor, char xy)
{
	for(unsigned int i = 0 ;  i<pts.size() ; i++)
	{
		strech(pts[i] ,Updated_pts[i] ,origin ,factor, xy);
	}
}


void guiUtils::normVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts)
{
    short maxPointX = pts[0].X;
    short minPointX = pts[0].X;
	short maxPointY = pts[0].Y;
	short minPointY = pts[0].Y;
    Point tempPoint(0,0);

	for(unsigned int i=0 ; i<pts.size() ; ++i)
	{
        maxPointX = ( pts[i].X > maxPointX ) ? pts[i].X : maxPointX;
        minPointX = ( pts[i].X < minPointX ) ? pts[i].X : maxPointX;
        maxPointY = ( pts[i].X > maxPointY ) ? pts[i].Y : maxPointY;
        minPointY = ( pts[i].X < minPointY ) ? pts[i].Y : maxPointY;

	}
  if(maxPointX - minPointX == 0) printf("Error devision by 0 in function guiUtils::normVec");
	for(unsigned int i = 0 ;  i<pts.size() ; i++)
	{
		tempPoint.X = ( pts[i].X - minPointX )/(maxPointX - minPointX )*MAP_FRAME_WIDTH + MAP_FRAME_POS_X;
		tempPoint.Y = ( pts[i].Y - minPointY )/(maxPointY - minPointY )*MAP_FRAME_LENGTH + MAP_FRAME_POS_Y;
		Updated_pts.push_back(tempPoint);
	}
}


void guiUtils::translateVec(std::vector<Point>& pts ,std::vector<Point>& Updated_pts ,Point& deltaXY)
{
	for(unsigned int i = 0 ;  i<pts.size() ; i++)
	{
		Updated_pts[i].X = pts[i].X + deltaXY.X;
		Updated_pts[i].Y = pts[i].Y + deltaXY.Y;
	}
}


//GPS Sampling functions
bool guiUtils::inNeighbourhood(Point& p1, Point& p2, double radius)
{
	short r = (short)radius;
	short d = (short)sqrt(((p1.X-p2.X)*(p1.X-p2.X)) + ((p1.Y-p2.Y)*(p1.Y-p2.Y)));
	if ( d < r ) return true;

	return false;
}

double guiUtils::velocityAng(std::vector<double>& velocity)
{
	double speed = sqrt( velocity[0]*velocity[0] + velocity[1]*velocity[1]);
	if(speed > 0)
		return 180/PI*atan2(velocity[0],velocity[1]);
	printf("Somting Wong.. Speed is 0! Call chinese guy to fix! or check in function guiUtils::velocityAng");
	return 0;
}

bool guiUtils::sampleNewPoint(std::vector<double>& vecLatitude,std::vector<double>& vecLongitude,double newLat,double newLon, unsigned int& numLastPointsInRadius)
{
	bool newPointSampled = false;
	//Case no points samples yet.
	if(vecLatitude.size() == 0){
    	numLastPointsInRadius++;
			vecLatitude.push_back(newLat);
			vecLongitude.push_back(newLon);
			newPointSampled = true;
	}
	else{
	//At least one point was already sampled.

	//Check if sampling criterion is met. In this case extra point must exceed sampling raduis to previous point.
		double prevLat = vecLatitude[vecLatitude.size()-1];
		double prevLon = vecLongitude[vecLongitude.size()-1];
		Point prevPoint(0,0);
		Point nextPoint(0,0);

    gps2linDist(prevPoint ,prevLat, prevLon);
		gps2linDist(nextPoint ,newLat, newLat);

		if(!(inNeighbourhood(prevPoint, nextPoint, SAMPLE_RADIUS))){
			vecLatitude.push_back(newLat);
			vecLongitude.push_back(newLon);
			numLastPointsInRadius++;
			newPointSampled = true;
		}
		else{
            //while in the neighbourhood keep averging out the last point.
            unsigned int defaultFilterWidth = (FILTER_WIDTH < numLastPointsInRadius) ? FILTER_WIDTH : numLastPointsInRadius;
            vecLatitude[vecLatitude.size()-1] = movingAveragefilter(vecLatitude,defaultFilterWidth,newLat);
            vecLongitude[vecLongitude.size()-1] = movingAveragefilter(vecLongitude,defaultFilterWidth,newLon);
            numLastPointsInRadius = 1;
            newPointSampled = false;
		}
	}
	return newPointSampled;
}
void guiUtils::gps2frame(std::vector<double>& vecLatitude,std::vector<double>& vecLongitude,std::vector<Point>& FramePts)
{
    std::vector<Point> gpsPts;
    Point tempPoint(0,0);
    //Latitiude & Longitude to linear distance of all points collected so far.
    for (unsigned int i=0 ; i<vecLatitude.size() ; i++)
    {
        gps2linDist(tempPoint ,vecLatitude[i], vecLongitude[i]);
        gpsPts.push_back(tempPoint);
    }
    //Normalize points to frame.
    normVec(gpsPts ,FramePts);
}

//MAP UTILITIES

void guiUtils::zoomMap(std::vector<Point>& pts ,std::vector<Point>& Updated_pts, double factor ,Point origin)
{
    strechVec(pts ,Updated_pts, origin ,factor, 'x');
    strechVec(pts ,Updated_pts, origin ,factor, 'y');
}

void guiUtils::setOrientation(std::vector<Point> originalPts, std::vector<Point> mapPts , std::vector<double> velocity)
{
		Point origin(VEHICLE_ORIGIN_X,VEHICLE_ORIGIN_Y);
		rotateVec(originalPts ,mapPts, origin ,-velocityAng(velocity));
} 
void guiUtils::setPosition(std::vector<Point> mapPts , Point prev_location,Point next_location)
{
		short delX = next_location.X - prev_location.X;
		short delY = next_location.Y - prev_location.Y;
		Point delXY(0,0);
		delXY.X = delX;
		delXY.Y = delY;
		Point origin(VEHICLE_ORIGIN_X,VEHICLE_ORIGIN_Y);
		translateVec(mapPts ,mapPts ,delXY);
}
void guiUtils::setScale(std::vector<Point> originalPts, std::vector<Point> mapPts , std::vector<double> velocity)
{
		double speed = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
		double factor = (200-speed)/100;
		Point origin(VEHICLE_ORIGIN_X,VEHICLE_ORIGIN_Y);
		zoomMap(originalPts ,mapPts, factor ,origin);
}

void guiUtils::buildMap(std::vector<double>& vecLatitude ,std::vector<double>& vecLongitude, double newLat, double newLon ,std::vector<Point> originalPts ,std::vector<Point> mapPts,unsigned int& numLastPointsInRadius,std::vector<double> velocity)
{
		Point prev_location = originalPts[originalPts.size()-1];
    Point next_location = originalPts[originalPts.size()-1];
    //Sample new Point
    if(sampleNewPoint(vecLatitude,vecLongitude,newLat,newLon,numLastPointsInRadius))
    { 
    	//Set Points to frame
    	gps2frame(vecLatitude,vecLongitude,originalPts);
    	//Set position relative to current location
    	if(vecLatitude.size()>1)
    	{
    		prev_location = originalPts[originalPts.size()-2];
			}
			setPosition(mapPts ,prev_location,next_location);
    	//Set Orientation relative to current velocity
			setOrientation(originalPts, mapPts , velocity);
    	//Reset to frame
    	gps2frame(vecLatitude,vecLongitude,originalPts);
    	//Set scale relaitive to speed
    	setScale(originalPts, mapPts ,velocity);
    }
}
