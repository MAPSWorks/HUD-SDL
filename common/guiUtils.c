/***** Utilities for gui ****/



//Local defines
#define SAMPLE_RADIUS 3
#define NOISE_RADIUS 56
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

/*double guiUtils::movingAveragefilter(std::vector<double>& vectorIN,double newTerm,unsigned int num_terms)
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
*/

Point::Point(int X, int Y) : X(X), Y(Y) {};
Coordinate::Coordinate(double X,double Y) : X(X), Y(Y) {};
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

void guiUtils::gps2linDist(Coordinate& updated_coordinate ,double lat, double lon)
{
	updated_coordinate.X = Earth_Radius*cos(lat/180.0*PI)*cos(lon/180.0*PI);
	updated_coordinate.Y = Earth_Radius*cos(lat/180.0*PI)*sin(lon/180.0*PI);
	//printf("updated coordinate (X,Y) = (%f,%f)\n",updated_coordinate.X , updated_coordinate.Y);
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


void guiUtils::normVec(std::vector<Coordinate>& pts ,std::vector<Point>& FramePts)
{
    Point tempPoint(MAP_FRAME_POS_X,MAP_FRAME_POS_Y);
    printf("tempPointInitialization (%d,%d)\n",tempPoint.X,tempPoint.Y);
    if(pts.size()==0){return;}
    else if(pts.size()==1){FramePts.push_back(tempPoint);}
    else{
    //Find Map borders
        double maxX=pts[0].X;
        double maxY=pts[0].Y;
        double minX=pts[0].X;
        double minY=pts[0].Y;
        for(unsigned int i=0 ; i<pts.size() ; ++i){
            maxX = (maxX > pts[i].X) ? maxX:pts[i].X;
            maxY = (maxY > pts[i].Y) ? maxY:pts[i].Y;
            minX = (minX < pts[i].X) ? minX:pts[i].X;
            minY = (minY < pts[i].Y) ? minY:pts[i].Y;
        }
        printf("maxX,minX (%f,%f) maxY,minY (%f,%f)\n",maxX,minX,maxY,minY);
        printf("maxX-minX %f maxY-minY %f\n",maxX-minX,maxY-minY);
        //Refresh points to newely normalized map
        for(unsigned int i=0 ; i<pts.size()-1 ; ++i){
            if(maxX>minX){
                tempPoint.X = (pts[i].X - minX)/(maxX - minX)*MAP_FRAME_LENGTH + MAP_FRAME_POS_X;
                }
            if(maxY>minY){
                tempPoint.Y = -(pts[i].Y - minY)/(maxY - minY)*MAP_FRAME_WIDTH + MAP_FRAME_POS_Y;
                }
            FramePts[i] = tempPoint;
        }
        if(maxX>minX){
                tempPoint.X = (pts[pts.size()-1].X - minX)/(maxX - minX)*MAP_FRAME_LENGTH + MAP_FRAME_POS_X;
                }
        if(maxY>minY){
                tempPoint.Y = -(pts[pts.size()-1].Y - minY)/(maxY - minY)*MAP_FRAME_WIDTH + MAP_FRAME_POS_Y;
                }
        FramePts.push_back(tempPoint);
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
bool guiUtils::inNeighbourhood(Coordinate& p1, Coordinate& p2, double radius)
{
	double r = radius;
	double d = sqrt(((p1.X-p2.X)*(p1.X-p2.X)) + ((p1.Y-p2.Y)*(p1.Y-p2.Y)));
	if ( d < r ) return true;

	return false;
}

bool guiUtils::isNoiseSample(Coordinate& p1, Coordinate& p2, double radius)
{
	double r = radius;
	double d = sqrt(((p1.X-p2.X)*(p1.X-p2.X)) + ((p1.Y-p2.Y)*(p1.Y-p2.Y)));
	if ( d > r ) return true;

	return false;
}

double guiUtils::angle(std::vector<double>& vec1 , std::vector<double>& vec2)
{
		return 180/PI*atan2(vec2[0]-vec1[0] ,vec2[1]-vec1[1]);
}

bool guiUtils::sampleNewPoint(std::vector<VnVector3>& vecVelocity,VnVector3& vel, std::vector<double>& vecLatitude,std::vector<double>& vecLongitude,double newLat,double newLon)
{
    //printf("test1\n");
    bool newPointSampled = false;
	//Case no points samples yet.
    if(newLat == 0 || newLon == 0) {
    //printf("test2\n");
    //No signal
    printf("No GPS Signal \n");
    return false;
    }

	else{
        //printf("test3\n");
        if(vecLatitude.size()==0){
            //printf("test4_start\n");
			vecLatitude.push_back(newLat);
			vecLongitude.push_back(newLon);
			vecVelocity.push_back(vel);
			newPointSampled = true;
			//printf("test4-End\n");
        }
        else{
            //printf("test5\n");
            //printf("Aquired GPS Signal \n");
            //At least one point was already sampled.

            //Check if sampling criterion is met. In this case extra point must exceed sampling raduis to previous point.
            double prevLat = vecLatitude[vecLatitude.size()-1];
            double prevLon = vecLongitude[vecLongitude.size()-1];
            Coordinate prevCoordinate(0,0);
            Coordinate nextCoordinate(0,0);
            Point zero(0,0);
            //printf("prevLat = %f prevLon = %f\n" , prevLat,prevLon);
            //printf("newLat = %f newLon = %f\n" , newLat,newLon);
            gps2linDist(prevCoordinate ,prevLat, prevLon);
            gps2linDist(nextCoordinate ,newLat, newLon);

            //printf("test5-end\n");

            if(!(isNoiseSample(prevCoordinate, nextCoordinate, NOISE_RADIUS)) && !(inNeighbourhood(prevCoordinate, nextCoordinate, SAMPLE_RADIUS))){
            //printf("test6\n");
                vecLatitude.push_back(newLat);
                vecLongitude.push_back(newLon);
                vecVelocity.push_back(vel);
                newPointSampled = true;
                //printf("new point sampled newLat = %f , newLon = %f\n",newLat,newLon);
            }
            else{
                //printf("test8\n");
                //while in the neighbourhood keep averging out the last point.
                //printf("prev = (%f,%f) next = (%f,%f)\n",prevCoordinate.X,prevCoordinate.Y,nextCoordinate.X,nextCoordinate.Y);
                if(isNoiseSample(prevCoordinate, nextCoordinate, NOISE_RADIUS)) {printf("Noise Detected\n");}
                vecLatitude[vecLatitude.size()-1] = (prevLat + newLat) / 2;
                vecLongitude[vecLongitude.size()-1] = (prevLon + newLon) / 2;

                vecVelocity[vecVelocity.size()-1].c0 = (vel.c0 + vecVelocity[vecVelocity.size()-1].c0) / 2;
                vecVelocity[vecVelocity.size()-1].c1 = (vel.c1 + vecVelocity[vecVelocity.size()-1].c1) / 2;
                vecVelocity[vecVelocity.size()-1].c2 = (vel.c2 + vecVelocity[vecVelocity.size()-1].c2) / 2;
                newPointSampled = false;
                //printf("(%f,%f) pussy in the neighbourhood\n" ,vecLatitude[vecLatitude.size()-1] ,vecLongitude[vecLatitude.size()-1]);
                //printf("test9\n");
            }
		}
	}
	return newPointSampled;
}


void guiUtils::gps2frame(std::vector<double>& vecLatitude,std::vector<double>& vecLongitude,std::vector<Point>& FramePts)
{
    //printf("fag1\n");
    std::vector<Coordinate> gpsPts;
    Coordinate tempCoordinate(0,0);
    //Latitiude & Longitude to linear distance of all points collected so far.
    //printf("fag2\n");
    for (unsigned int i=0 ; i<vecLatitude.size() ; i++)
    {
        //printf("fag3\n");
        gps2linDist(tempCoordinate ,vecLatitude[i], vecLongitude[i]);
        //printf("fag4\n");
        gpsPts.push_back(tempCoordinate);
        //printf("fag5\n");
    }
    //Normalize points to frame.
    //printf("fag6\n");
    normVec(gpsPts ,FramePts);
    for (unsigned int i = 0; i<FramePts.size();++i){
        printf("FramePts = (%d,%d) size = %d\n",FramePts[i].X,FramePts[i].Y,FramePts.size());
        printf("GPSPts = (%f,%f) size = %d\n",gpsPts[i].X,gpsPts[i].Y,gpsPts.size());
        }
}

/*************************Map Utilities********************************/

void guiUtils::zoomMap(std::vector<Point>& pts ,std::vector<Point>& Updated_pts, double factor ,Point origin)
{
    strechVec(pts ,Updated_pts, origin ,factor, 'x');
    strechVec(pts ,Updated_pts, origin ,factor, 'y');
}

void guiUtils::setOrientation(std::vector<Point> originalPts, std::vector<Point> mapPts , std::vector<double> prevVelocity,std::vector<double> nextVelocity)
{
		Point origin(VEHICLE_ORIGIN_X,VEHICLE_ORIGIN_Y);
		rotateVec(mapPts ,mapPts, origin ,-angle(prevVelocity , nextVelocity));
}
void guiUtils::setPosition(std::vector<Point> mapPts , Point prev_location,Point next_location)
{
		short delX = next_location.X - prev_location.X;
		short delY = next_location.Y - prev_location.Y;
		Point delXY(0,0);
		delXY.X = delX;
		delXY.Y = delY;
		Point origin(RELATIVE_PLACE_ARROW_X,RELATIVE_PLACE_ARROW_Y);
		translateVec(mapPts ,mapPts ,delXY);
}
void guiUtils::setScale(std::vector<Point> originalPts, std::vector<Point> mapPts , std::vector<double> velocity)
{
		double speed = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
		double factor = (MAX_SPEED-speed)/AVG_SPEED;
		Point origin(RELATIVE_PLACE_ARROW_X,RELATIVE_PLACE_ARROW_Y);
		zoomMap(mapPts ,mapPts, factor ,origin);
}

void guiUtils::buildMap(std::vector<VnVector3>& vecVelocity,std::vector<double>& vecLatitude ,std::vector<double>& vecLongitude, double newLat, double newLon,std::vector<Point>& originalPts,VnVector3 velocity)
{
    if(sampleNewPoint(vecVelocity,velocity,vecLatitude,vecLongitude,newLat,newLon))
        gps2frame(vecLatitude,vecLongitude,originalPts);
}

void guiUtils::UpdateMap(std::vector<Point>& originalPts ,std::vector<Point>& mapPts ,std::vector<VnVector3>& vecVelocity)
{
    Point prev_location(0,0);
    Point next_location(0,0);
    Point delLocation(0,0);
    std::vector<double> nextVelocity = {0};
    std::vector<double> prevVelocity = {0};
    std::vector<double> zero3 = {0};
    Point zero(0,0);

    Point origin(RELATIVE_PLACE_ARROW_X,RELATIVE_PLACE_ARROW_Y);

    if(originalPts.size()>1)
    {
        nextVelocity[0] = (double)vecVelocity[vecVelocity.size()-1].c0;
        nextVelocity[1] = (double)vecVelocity[vecVelocity.size()-1].c1;
        nextVelocity[2] = (double)vecVelocity[vecVelocity.size()-1].c2;

        prevVelocity[0] = (double)vecVelocity[vecVelocity.size()-2].c0;
        prevVelocity[1] = (double)vecVelocity[vecVelocity.size()-2].c1;
        prevVelocity[2] = (double)vecVelocity[vecVelocity.size()-2].c2;

        prev_location = mapPts[mapPts.size()-2];
        next_location = mapPts[mapPts.size()-1];

        delLocation.X = prev_location.X;
        delLocation.Y = prev_location.Y;
        rotatePoint(delLocation,delLocation,zero,-angle(prevVelocity,nextVelocity));

        setPosition(mapPts,origin,delLocation);
        setOrientation(originalPts, mapPts , prevVelocity ,nextVelocity);
    }


    setScale(originalPts ,mapPts ,nextVelocity);
}

/******************Line of sight functions************************/

//void skewPts()
//{

//}
//void setTrailInLineOfSight(std::vector<Point> mapPts)
//{
//    unsigned int upperIndx = (mapPts.size() < MAX_PTS_IN_LINE_OF_SIGHT) ? mapPts.size() : MAX_PTS_IN_LINE_OF_SIGHT;
    //Duplicate at most MAX_PTS_IN_LINE_OF_SIGHT points to either side of the field of view

    //Vertical projection according to height
//}



