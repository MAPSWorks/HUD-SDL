/***** Utilities for gui ****/



//Local defines
#define SAMPLE_RADIUS 3
#define LOOP_DETECTION_RADIUS 4
#define NOISE_RADIUS 56
#define FILTER_WIDTH 10
#define VEHICLE_ORIGIN_X 0
#define VEHICLE_ORIGIN_Y 0
#define THRESHHOLD_ANGLE 15

//Local includes
#include "gui.h"
#include <iostream>
#include <vector>
#include <math.h>
#include "sensorenv.h"

//General

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


void guiUtils::normVec(std::vector<Coordinate>& pts ,std::vector<Point>& FramePts,bool frameDef,std::vector<double>& frame)
{
    Point tempPoint(MAP_FRAME_POS_X,MAP_FRAME_POS_Y);
    double scale;
    //printf("test1\n");
    if(pts.size()==0)
    {
        //printf("test2\n");
        return;
    }
    else if(pts.size()==1){
        //printf("test3\n");
        FramePts.push_back(tempPoint);
    }
    else{
    //Find Map borders
    //printf("test4\n");
    if(!frameDef)
    {
        //printf("test5\n");
        frame.clear();
        frame.push_back(pts[0].X);//maxX
        frame.push_back(pts[0].Y);//maxY
        frame.push_back(pts[0].X);//minX
        frame.push_back(pts[0].Y);//minY
    }
    else
    {
        //printf("test6 size pts = %d , size frame = %d\n",pts.size(),frame.size());
        frame[0] = (frame[0] > pts[0].X) ? frame[0]:pts[0].X;//maxX
        frame[1] = (frame[1] > pts[0].Y) ? frame[1]:pts[0].Y;//maxY
        frame[2] = (frame[2] < pts[0].X) ? frame[2]:pts[0].X;//minX
        frame[3] = (frame[3] < pts[0].Y) ? frame[3]:pts[0].Y;//minY
        //printf("test6Pass\n");
    }
    for(unsigned int i=0 ; i<pts.size() ; ++i)
    {
        //printf("test7\n");
        frame[0] = (frame[0] > pts[i].X) ? frame[0]:pts[i].X;
        frame[1] = (frame[1] > pts[i].Y) ? frame[1]:pts[i].Y;
        frame[2] = (frame[2] < pts[i].X) ? frame[2]:pts[i].X;
        frame[3] = (frame[3] < pts[i].Y) ? frame[3]:pts[i].Y;
    }
    scale = (frame[0] - frame[2] > frame[1] - frame[3]) ? (frame[0] - frame[2]) : (frame[1] - frame[3]);
    for(unsigned int i=0 ; i<pts.size()-1 ; ++i){
        //printf("test8\n");
        if(frame[0]>frame[2]){
            tempPoint.X = (pts[i].X - frame[2])/scale*MAP_FRAME_LENGTH + MAP_FRAME_POS_X;
            }
        if(frame[1]>frame[3]){
            tempPoint.Y = -(pts[i].Y - frame[3])/scale*MAP_FRAME_WIDTH + MAP_FRAME_POS_Y;
            }
        FramePts[i] = tempPoint;
    }
    if(frame[0]>frame[2]){
            //printf("test9\n");
            tempPoint.X = (pts[pts.size()-1].X - frame[2])/scale*MAP_FRAME_LENGTH + MAP_FRAME_POS_X;
            }
    if(frame[1]>frame[3]){
            //printf("test10\n");
            tempPoint.Y = -(pts[pts.size()-1].Y - frame[3])/scale*MAP_FRAME_WIDTH + MAP_FRAME_POS_Y;
            }
    //printf("test11\n");
    if(pts.size() > FramePts.size())
        FramePts.push_back(tempPoint);
    else if(pts.size() == FramePts.size())
        FramePts[FramePts.size()-1] = tempPoint;
    else
        printf("Error in guiUtils::normVec() Frame pts size exceeds actual number of sampled points");
    }
    //printf("test12\n");
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

double guiUtils::VnAngle(VnVector3 vec1 ,VnVector3 vec2)
{
		return 180/PI*atan2((double)vec2.c0-(double)vec1.c0 ,(double)vec2.c1-(double)vec1.c1);
}

double guiUtils::CoordinateAngle(Coordinate p1 ,Coordinate p2)
{
		return 180/PI*atan2(p1.X-p2.X ,p1.Y-p2.Y);
}

double guiUtils::pointAngle(Point p1 ,Point p2)
{
		return 180/PI*atan2((double)p1.X-(double)p2.X ,(double)p1.Y-(double)p2.Y);
}


bool guiUtils::sampleNewPoint(std::vector<VnVector3>& vecVelocity,VnVector3& vel, std::vector<double>& vecLatitude,std::vector<double>& vecLongitude,double newLat,double newLon)
{
    //printf("test1\n");
    bool newPointSampled = false;
	//Case no points samples yet.
    if(newLat == 0 || newLon == 0) {
    //No signal
    printf("No GPS Signal \n");
    return false;
    }

	else{
        if(vecLatitude.size()==0){
			vecLatitude.push_back(newLat);
			vecLongitude.push_back(newLon);
			vecVelocity.push_back(vel);
			newPointSampled = true;
        }
        else{

            double prevLat = vecLatitude[vecLatitude.size()-1];
            double prevLon = vecLongitude[vecLongitude.size()-1];
            Coordinate prevCoordinate(0,0);
            Coordinate nextCoordinate(0,0);
            Point zero(0,0);

            gps2linDist(prevCoordinate ,prevLat, prevLon);
            gps2linDist(nextCoordinate ,newLat, newLon);


            if(!(isNoiseSample(prevCoordinate, nextCoordinate, NOISE_RADIUS)) && !(inNeighbourhood(prevCoordinate, nextCoordinate, SAMPLE_RADIUS))){

                vecLatitude.push_back(newLat);
                vecLongitude.push_back(newLon);
                vecVelocity.push_back(vel);
                newPointSampled = true;

            }
            else{

                if(isNoiseSample(prevCoordinate, nextCoordinate, NOISE_RADIUS)) {printf("Noise Detected\n");}
                vecLatitude[vecLatitude.size()-1] = (prevLat + newLat) / 2;
                vecLongitude[vecLongitude.size()-1] = (prevLon + newLon) / 2;

                vecVelocity[vecVelocity.size()-1].c0 = (vel.c0 + vecVelocity[vecVelocity.size()-1].c0) / 2;
                vecVelocity[vecVelocity.size()-1].c1 = (vel.c1 + vecVelocity[vecVelocity.size()-1].c1) / 2;
                vecVelocity[vecVelocity.size()-1].c2 = (vel.c2 + vecVelocity[vecVelocity.size()-1].c2) / 2;
                newPointSampled = false;

            }
		}
	}
	return newPointSampled;
}


void guiUtils::gps2frame(std::vector<double>& vecLatitude,std::vector<double>& vecLongitude,std::vector<Point>& FramePts,bool frameDef,std::vector<double>& frame)
{
    //printf("testGPSframe1");
    std::vector<Coordinate> gpsPts;
    Coordinate tempCoordinate(0,0);
    //Latitiude & Longitude to linear distance of all points collected so far.
    //printf("testGPSframe2");
    for (unsigned int i=0 ; i<vecLatitude.size() ; i++)
    {
        //printf("testGPSframe3");
        gps2linDist(tempCoordinate ,vecLatitude[i], vecLongitude[i]);
        //printf("testGPSframe4");
        gpsPts.push_back(tempCoordinate);
        //printf("testGPSframe5");
    }
    //Normalize points to frame.
    //printf("testGPSframe6");
    normVec(gpsPts ,FramePts,frameDef,frame);
    //printf("testGPSframe7");
}

/*************************Map Utilities********************************/

void guiUtils::zoomMap(std::vector<Point>& pts ,std::vector<Point>& Updated_pts, double factor ,Point origin)
{
    strechVec(pts ,Updated_pts, origin ,factor, 'x');
    strechVec(pts ,Updated_pts, origin ,factor, 'y');
}


bool guiUtils::buildMap(std::vector<VnVector3>& vecVelocity,std::vector<double>& vecLatitude ,std::vector<double>& vecLongitude, double newLat, double newLon,std::vector<Point>& originalPts,VnVector3 velocity,bool frameDef,std::vector<double>& frame)
{
    if(sampleNewPoint(vecVelocity,velocity,vecLatitude,vecLongitude,newLat,newLon))
    {
        gps2frame(vecLatitude,vecLongitude,originalPts,frameDef,frame);
        return true;
    }
    return false;
}


int guiUtils::isClosedLoop(std::vector <VnVector3>& vecVelocity,std::vector<double>& vecLatitude,std::vector<double>& vecLongitude)
{
    if(vecLatitude.size()>SIZE_TRAIL)
    {
        //printf("test1\n");
        Coordinate currCo(0,0);
        VnVector3 currVel = vecVelocity[vecVelocity.size()-1];
        Coordinate tempCo(0,0);
        VnVector3 tempVel = {0};
        gps2linDist(currCo,vecLatitude[vecLatitude.size()-1],vecLongitude[vecLongitude.size()-1]);
        //printf("test2\n");
        for(unsigned int i=0 ; i<vecLatitude.size()-SIZE_TRAIL ; ++i)
        {
            //printf("test3\n");
            gps2linDist(tempCo,vecLatitude[i],vecLongitude[i]);
            //printf("*****tempCo-currCo = (%f,%f)******\n" ,tempCo.X-currCo.X,tempCo.Y-currCo.Y);
            //printf("*****tempVel-currVel = (%f,%f)******\n" ,tempVel.c0-currVel.c0,tempVel.c1-currVel.c1);
            if(inNeighbourhood(tempCo,currCo,LOOP_DETECTION_RADIUS))
            {   //printf("test4\n");
                tempVel = vecVelocity[i];
                if(vnSpeed(tempVel)==0 || vnSpeed(currVel)==0)
                {
                //printf("test5\n Angle = %f\n",VnAngle(vnHat(tempVel),vnHat(currVel)));
                //printf("test5\n Speed(temp,curr) = (%f,%f)\n",vnSpeed(tempVel),vnSpeed(currVel));
                continue;/**Test here for trail distortion to reject Loop completion test!!!**/
                }
                else if(VnAngle(vnHat(tempVel),vnHat(currVel))<THRESHHOLD_ANGLE)
                {
                    //printf("test6\n");
                    return i; //in case of OverShoot.
                }
            }
        }
    }
    //printf("test7\n");
    return -1;
}

double guiUtils::vnSpeed(VnVector3 vel)
{
    return sqrt(vel.c0*vel.c0+vel.c1*vel.c1);
}

VnVector3 guiUtils::vnHat(VnVector3 vec)
{
    VnVector3 temp = {0};
    if(vnSpeed(vec)>0){
        temp.c0 = vec.c0/vnSpeed(vec);
        temp.c1 = vec.c1/vnSpeed(vec);
        }
    return temp;
}

void guiUtils::UpdateMap(std::vector<Point>& originalPts,std::vector<Point>& mapPts,std::vector<VnVector3> vecVelocity,Point origin, Point deltaXY,bool newPointSampled)
{
    //Curr map
    VnVector3 vnZero = {0};
    mapPts.clear();
    //printf("size = %d\n",originalPts.size());
    for(unsigned int i=0 ; i<originalPts.size() ; ++i)
    {
        mapPts.push_back(originalPts[i]);
    }
    if(originalPts.size()>0)
    {
        //deltaXY.X = MAP_FRAME_POS_X - originalPts[originalPts.size()-1].X ;
        //deltaXY.Y = MAP_FRAME_POS_Y - originalPts[originalPts.size()-1].Y ;
        translateVec(originalPts ,mapPts ,deltaXY);
        rotateVec(originalPts ,mapPts, origin ,0);
        printf("origin = (%d,%d) VnAngle = %f , delXY = (%d,%d)\n",origin.X,origin.Y,VnAngle(vecVelocity[vecVelocity.size()-1],vnZero),deltaXY.X,deltaXY.Y);
    }
}

/******************Line of sight functions************************/



