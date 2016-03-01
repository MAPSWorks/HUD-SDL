/***** Utilities for gui ****/
//I love Salmon!! It's fucking amazing full of protein, low fat and tastes like heaven!!!
//I also like a round ass & samll tits (: Yes I do! Home to find them some day..


//Local defines
#define SAMPLE_RADIUS 1

#define LOOP_DETECTION_RADIUS 4
#define NOISE_RADIUS 100
#define FILTER_WIDTH 10
#define VEHICLE_ORIGIN_X 0
#define VEHICLE_ORIGIN_Y 0
#define THRESHHOLD_ANGLE 18
#define QUATERNION_SINGULARITY_TH 0.999999
//Local includes
#include "gui.h"
#include <iostream>
#include <vector>
#include <math.h>
#include "sensorenv.h"

//General
extern SDL_Renderer** globgRenderer;

Point::Point(void) : X(0), Y(0) {};
Point::Point(int X, int Y) : X(X), Y(Y) {};
Coordinate::Coordinate(double X,double Y) : X(X), Y(Y) {};
//SUB UTILITIES
//point functions

bool guiUtils::gpsSinal(Coordinate co)
{
    if(co.X==0 || co.Y==0)
    {
        //printf("No GPS signal");
        return false;
    }
    return true;
}

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
    //printf("d = %f\n",d);
	if ( d < r ) return true;

	return false;
}

bool guiUtils::isNoiseSample(Coordinate& p1, Coordinate& p2, double radius)
{
	double r = radius;
	double d = sqrt(((p1.X-p2.X)*(p1.X-p2.X)) + ((p1.Y-p2.Y)*(p1.Y-p2.Y)));
	//printf("d = %f\n",d);
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


bool guiUtils::sampleNewPoint(std::vector<VnVector3>& vecVelocity,VnVector3& vel, std::vector<double>& vecLatitude,std::vector<double>& vecLongitude ,std::vector<double>& vecAltitude,double newLat,double newLon,double newAlt)
{
    //printf("test1\n");
    bool newPointSampled = false;
	//Case no points samples yet.
    if(newLat == 0 || newLon == 0) {
    //No signal
    //printf("No GPS Signal \n");
    //gpsSig = false;
    return false;
    }

	else{
        if(vecLatitude.size()==0){
			vecLatitude.push_back(newLat);
			vecLongitude.push_back(newLon);
			vecAltitude.push_back(newAlt);
			vecVelocity.push_back(vel);
			newPointSampled = true;
        }
        else{

            double prevLat = vecLatitude[vecLatitude.size()-1];
            double prevLon = vecLongitude[vecLongitude.size()-1];
            double prevAlt = vecAltitude[vecAltitude.size()-1];
            Coordinate prevCoordinate(0,0);
            Coordinate nextCoordinate(0,0);
            Point zero(0,0);

            gps2linDist(prevCoordinate ,prevLat, prevLon);
            gps2linDist(nextCoordinate ,newLat, newLon);


            if(!(isNoiseSample(prevCoordinate, nextCoordinate, NOISE_RADIUS)) && !(inNeighbourhood(prevCoordinate, nextCoordinate, SAMPLE_RADIUS))){

                vecLatitude.push_back(newLat);
                vecLongitude.push_back(newLon);
                vecAltitude.push_back(newAlt);
                vecVelocity.push_back(vel);
                newPointSampled = true;

            }
            else{

                if(isNoiseSample(prevCoordinate, nextCoordinate, NOISE_RADIUS)) {printf("Noise Detected\n");}
                vecLatitude[vecLatitude.size()-1] = (prevLat + newLat) / 2;
                vecLongitude[vecLongitude.size()-1] = (prevLon + newLon) / 2;
                vecAltitude[vecAltitude.size()-1] = (prevAlt + newAlt) / 2;

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


bool guiUtils::buildMap(std::vector<VnVector3>& vecVelocity,std::vector<double>& vecLatitude ,std::vector<double>& vecLongitude,std::vector<double>& vecAltitude, double newLat, double newLon,double newAlt,std::vector<Point>& originalPts,VnVector3 velocity,bool frameDef,std::vector<double>& frame)
{
    if(sampleNewPoint(vecVelocity,velocity,vecLatitude,vecLongitude,vecAltitude,newLat,newLon,newAlt))
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
//int trailDist()

double guiUtils::vnSpeed(VnVector3 vel)
{
    return 1/sqrt(FP_PRECITION)*sqrt(FP_PRECITION*vel.c0*vel.c0+FP_PRECITION*vel.c1*vel.c1);
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

void guiUtils::UpdateMap(std::vector<Point>& originalPts,std::vector<Point>& mapPts,std::vector<VnVector3> vecVelocity,Point origin, Point deltaXY,bool newPointSampled,double angRot)
{
    //Curr map
    //VnVector3 vnZero = {0};
    Point zero(0,0);
    mapPts.clear();

    //printf("size = %d\n",originalPts.size());
    for(unsigned int i=0 ; i<originalPts.size() ; ++i)
    {
        mapPts.push_back(originalPts[i]);
    }
    if(originalPts.size()>=1)
    {
        //deltaXY.X = MAP_FRAME_POS_X - originalPts[originalPts.size()-1].X ;
        //deltaXY.Y = MAP_FRAME_POS_Y - originalPts[originalPts.size()-1].Y ;
        //translateVec(originalPts ,mapPts ,deltaXY);
        //2 Cases a.Either Velocity is 0 b.size == 1
        //if(vnSpeed(vecVelocity[vecVelocity.size()])!=0 && vnSpeed(vecVelocity[vecVelocity.size()-1])!=0)
        //rotateVec(originalPts ,mapPts, origin , VnAngle(vecVelocity[vecVelocity.size()],vecVelocity[vecVelocity.size()-1]));
        rotateVec(originalPts ,mapPts, origin , angRot);
        translateVec(mapPts ,mapPts ,deltaXY);
        //std::cout << "vec length: " << vecVelocity.size() << std::endl;
        //std::cout << "last: (" << vecVelocity[vecVelocity.size()].c0 << "," << vecVelocity[vecVelocity.size()].c1 << std::endl;
        //printf("origin = (%d,%d) VnAngle = %f , delXY = (%d,%d)\n",origin.X,origin.Y,VnAngle(vecVelocity[vecVelocity.size()-1],vnZero),deltaXY.X,deltaXY.Y);
    }
}

/******************Line of sight functions************************/

bool guiUtils::vnRotate3D(double angDeg, VnVector3& vecIn, VnVector3& vecOut,char xyz)
{
    double theta = PI/180*angDeg;

    vecOut = {0};
    if(xyz=='x')
    {//Rx(theta)
        vecOut.c0 = vecIn.c0;
        vecOut.c1 = cos(theta)*vecIn.c1 - sin(theta)*vecIn.c2;
        vecOut.c2 = sin(theta)*vecIn.c1 + cos(theta)*vecIn.c2;
    }
    else if(xyz=='y')
    {//Ry(theta)
        vecOut.c0 = cos(theta)*vecIn.c0 + sin(theta)*vecIn.c2;
        vecOut.c1 = vecIn.c1;
        vecOut.c2 = -sin(theta)*vecIn.c0 + cos(theta)*vecIn.c2;
    }
    else if(xyz=='z')
    {//Rz(theta)
        vecOut.c0 = cos(theta)*vecIn.c0 - sin(theta)*vecIn.c1;
        vecOut.c1 = sin(theta)*vecIn.c0 + cos(theta)*vecIn.c1;
        vecOut.c2 = vecIn.c2;
    }
    else
    {
        printf("In function vnRotate3D invalid Input to function in at least one argument\n");
        return false;
        vecOut = vecIn;
    }
    return true;
}

void guiUtils::ypr2quat(double yaw,double pitch,double roll,std::vector<double>& quaternion)
{
     quaternion.clear();
     //roll and pitch exchaged due to sensor initial orientation
     double psy = PI/180*yaw;
     double theta = PI/180*roll;
     double phy = PI/180*pitch;
     quaternion.push_back(cos(phy/2)*cos(theta/2)*cos(psy/2)+sin(phy/2)*sin(theta/2)*sin(psy/2));
     quaternion.push_back(sin(phy/2)*cos(theta/2)*cos(psy/2)-cos(phy/2)*sin(theta/2)*sin(psy/2));
     quaternion.push_back(cos(phy/2)*sin(theta/2)*cos(psy/2)+sin(phy/2)*cos(theta/2)*sin(psy/2));
     quaternion.push_back(cos(phy/2)*cos(theta/2)*sin(psy/2)-sin(phy/2)*sin(theta/2)*cos(psy/2));
}

double guiUtils::quat2AngleAxis(std::vector<double> quaternion , std::vector<double>& axis)
{
    axis.clear();
    double angle = 0;
    if(quaternion[0]>QUATERNION_SINGULARITY_TH || quaternion[0]<-QUATERNION_SINGULARITY_TH)
    {
        axis.push_back(1);
        axis.push_back(0);
        axis.push_back(0);
        printf("In function quat2AngleAxis near singularity\n");
    }
    else
    {
        angle = acos(quaternion[0]);
        axis.push_back(quaternion[1]/sin(angle));
        axis.push_back(quaternion[2]/sin(angle));
        axis.push_back(quaternion[3]/sin(angle));
    }
    return 180/PI*2*angle;
}
bool guiUtils::isInScr(Point P)
{
    if(P.X >=0 && P.X<=SCREEN_WIDTH && P.Y>=0 && P.Y<=SCREEN_HEIGHT)
        return true;
    return false;
}
bool guiUtils::setCoordinateToScr(double lat0,double lon0,double alt0,double latP,double lonP,double altP,double yaw,double pitch,double roll,Point& scrP)
{
    Coordinate R0(0,0);
    Coordinate RP(0,0);
    std::vector<double> quaternion;
    std::vector<double> delR;
    std::vector<double> projAxisDelR;
    double scrRotAngDeg = 0;
    std::vector<double> axis;
    double projLength;
    Point zero(0,0);

    ypr2quat(-yaw,-pitch,-roll,quaternion);
    scrRotAngDeg = quat2AngleAxis(quaternion ,axis);
    gps2linDist(R0,lat0,lon0);
    gps2linDist(RP,latP,lonP);
    delR.push_back(RP.X-R0.X);
    delR.push_back(RP.Y-R0.Y);
    delR.push_back(altP-alt0);
    //Normalize
    double normR = sqrt(delR[0]*delR[0]+delR[1]*delR[1]+delR[2]*delR[2]);
    if(normR == 0) {
    printf("in function guiUtils::setCoordinateToScr error normR==0\n");
    //printf("in function guiUtils::setCoordinateToScr error normR==0\n");
    return false;
    }
    delR[0] = delR[0]/normR;
    delR[1] = delR[1]/normR;
    delR[2] = delR[2]/normR;
    //Project normal to screen axis
    projLength = delR[0]*axis[0] + delR[1]*axis[1] + delR[2]*axis[2];

    if(projLength<0){return false;} //Not in screen

    delR[0] = delR[0]/projLength;
    delR[1] = delR[1]/projLength;
    delR[2] = delR[2]/projLength;

    projAxisDelR.push_back(delR[0]-axis[0]);
    projAxisDelR.push_back(delR[1]-axis[1]);
    projAxisDelR.push_back(delR[2]-axis[2]);
    //transform to scr coordinates with an origin at the center
    rotate2XY(projAxisDelR,axis, scrP);

    rotatePoint(scrP,scrP,zero,scrRotAngDeg);

    if(isInScr(scrP))
    {
        printf("Point (%d,%d) on Screen\n",scrP.X,scrP.Y);
        return true;
    }
    printf("Point (%d,%d) not on screen\n",scrP.X,scrP.Y);
    return false;
}

void guiUtils::rotate2XY(std::vector<double>& vec2XY, std::vector<double>& normal, Point& scrP)
{
    VnVector3 vnVec2XY = {vec2XY[0],vec2XY[1],vec2XY[2]};
    VnVector3 vnVec2XYRotatedZ = {0};
    VnVector3 vnVec2XYRotatedZY = {0};
    double theta = 180/PI*atan2(normal[1],normal[0]);
    double Psy = 180/PI*atan2(normal[2],sqrt(normal[0]*normal[0]+normal[1]*normal[1]));
    vnRotate3D(theta, vnVec2XY, vnVec2XYRotatedZ ,'y');
    vnRotate3D(Psy, vnVec2XYRotatedZ, vnVec2XYRotatedZY ,'z');
    printf("normal = (%f,%f,%f)\n",normal[0],normal[1],normal[2]);
    //Debug
    VnVector3 vnNormal = {normal[0],normal[1],normal[2]};
    VnVector3 eZ = {0};
    VnVector3 eZY = {0};
    vnRotate3D(theta, vnNormal, eZ ,'z');
    vnRotate3D(Psy, eZ, eZY ,'y');
    //End denug
    printf("In function rotate2XY z e must be (0,0,1) e = (%f,%f,%f)\n",eZY.c0,eZY.c1,eZY.c2);
    printf("In function rotate2XY z component must be 0 Rz = %f\n",vnVec2XYRotatedZY.c2);
    scrP.X = SCREEN_WIDTH/2 + EYE_RELEIF/PHYSICAL_PIXEL_SIZE*vnVec2XYRotatedZY.c1;
    scrP.Y = SCREEN_HEIGHT/2 - EYE_RELEIF/PHYSICAL_PIXEL_SIZE*vnVec2XYRotatedZY.c2;
    printf("vnVector2XYRotatedZY = (%f,%f,%f)\n",vnVec2XYRotatedZY.c0,vnVec2XYRotatedZY.c1,vnVec2XYRotatedZY.c2);
    printf("vnVector2XY = (%f,%f,%f)\n",vnVec2XY.c0,vnVec2XY.c1,vnVec2XY.c2);

}

void guiUtils::renderTrail2scr(double lat0,double lon0,double alt0 ,std::vector<double>& vecLatitude_Prev,std::vector<double>& vecLongitude_Prev,std::vector<double>& vecAltitude_Prev,double yaw, double pitch, double roll, std::vector<Point>& scrPts)
{
    //printf("renderTrail2scr - Test1\n");
    Point scrP(0,0);
    scrPts.clear();
    double latP = 0;
    double lonP = 0;
    double altP = 0;
    //printf("renderTrail2scr - Test2\n");
    for(unsigned int i=0 ; i<vecLatitude_Prev.size() ; ++i)
    {
        //printf("renderTrail2scr - Test3\n");
        latP = vecLatitude_Prev[i];
        lonP = vecLongitude_Prev[i];
        altP = vecAltitude_Prev[i];
        if(coordinate2Scr(lat0,lon0,alt0,latP,lonP,altP,yaw,roll,pitch,scrP)&&isInScr(scrP)) //if Point is on screen bool->true
            scrPts.push_back(scrP);
    }
    //printf("renderTrail2scr - Test4\n");
}



void guiUtils::getPitchYawFromVec(std::vector<double>& vec, double& pitch,double& yaw)
{
    yaw = atan2(vec[0],-vec[1]);
    pitch = -atan2(vec[2],sqrt(vec[0]*vec[0]+vec[1]*vec[1]));
}

bool guiUtils::coordinate2Scr(double lat0,double lon0,double alt0,double latP,double lonP,double altP,double pitch,double yaw,double roll,Point& scrP)
{
    std::vector<double> delR;

    if(!coordinate2RelSpace(lat0,lon0,alt0,latP,lonP,altP,delR)){return false;}
    if(!relSpace2Scrn(delR,pitch,yaw,roll,scrP)){return false;}
    //printf("test coordinate2Scrn\n");
    //printf("scrP = (%d,%d)\n",scrP.X,scrP.Y);

    return true;
}

bool guiUtils::coordinate2RelSpace(double lat0,double lon0,double alt0,double latP,double lonP,double altP,std::vector<double>& delR)
{
    Coordinate R0(0,0);
    Coordinate RP(0,0);

    gps2linDist(R0,lat0,lon0);
    gps2linDist(RP,latP,lonP);
    delR.push_back(RP.X-R0.X);
    delR.push_back(RP.Y-R0.Y);
    delR.push_back(altP-alt0);

    if(sqrt(delR[0]*delR[0]+delR[1]*delR[1]+delR[2]*delR[2])==0) {return false;}
    return true;
}

bool guiUtils::onScrn(double pitch,double yaw)
{
    //printf("yawTh = %f \n",180/PI*atan(0.5));
    //printf("yp = (%f,%f)",yaw,pitch);
    if( (yaw < 180/PI*atan(0.5)) && (yaw > -180/PI*atan(0.5)))
    {
        return true;
    }
    return false;
}

bool guiUtils::relSpace2Scrn(std::vector<double>& delR,double pitch,double yaw,double roll,Point& scrP)
{
    double Rpitch;
    double Ryaw;
    Point origin(SCREEN_WIDTH/2,SCREEN_HEIGHT/2);


    getPitchYawFromVec(delR,Rpitch,Ryaw);
    //printf("Rpitch + PI/180*pitch = %f Ryaw - PI/180*yaw = %f\n" , 180/PI*Rpitch + pitch,180/PI*Ryaw - yaw );
    if(!onScrn(180/PI*Rpitch + pitch,180/PI*Ryaw - yaw)){return false;}

    scrP.Y = SCREEN_HEIGHT/2 + EYE_RELEIF/PHYSICAL_PIXEL_SIZE*tan(Rpitch + PI/180*pitch);
    scrP.X = SCREEN_WIDTH/2 + EYE_RELEIF/PHYSICAL_PIXEL_SIZE*tan(Ryaw - PI/180*yaw);
    rotatePoint(scrP,scrP,origin,90+roll);


    return true;
}

bool guiUtils::sideLines2Scrn(double pitch,double yaw,double roll,double lat0,double lon0,double alt0,double latP1,double lonP1,double altP1,double latP2,double lonP2,double altP2,Point& scrPLeft1, Point& scrPRight1,Point& scrPLeft2, Point& scrPRight2)
{
    std::vector<double> delR1;
    std::vector<double> delR2;
    std::vector<double> e12;
    std::vector<double> o12;

    std::vector<double> delR1_Left;
    std::vector<double> delR1_Right;
    std::vector<double> delR2_Left;
    std::vector<double> delR2_Right;

    if(!coordinate2RelSpace(lat0,lon0,alt0,latP1,lonP1,altP1,delR1)){return false;}
    if(!coordinate2RelSpace(lat0,lon0,alt0,latP2,lonP2,altP2,delR2)){return false;}

    e12.push_back(delR2[0] - delR1[0]);
    e12.push_back(delR2[1] - delR1[1]);

    e12[0] = e12[0]/sqrt(e12[0]*e12[0] + e12[1]*e12[1]);
    e12[1] = e12[1]/sqrt(e12[0]*e12[0] + e12[1]*e12[1]);

    o12.push_back(e12[0]*ROAD_WIDTH);
    o12.push_back(-e12[1]*ROAD_WIDTH);

    delR1_Left.push_back(delR1[0] + o12[0]);
    delR1_Left.push_back(delR1[1] + o12[1]);
    delR1_Left.push_back(delR1[2]);

    delR1_Right.push_back(delR1[0] - o12[0]);
    delR1_Right.push_back(delR1[1] - o12[1]);
    delR1_Right.push_back(delR1[2]);

    delR2_Left.push_back(delR2[0] + o12[0]);
    delR2_Left.push_back(delR2[1] + o12[1]);
    delR2_Left.push_back(delR2[2]);

    delR2_Right.push_back(delR2[0] - o12[0]);
    delR2_Right.push_back(delR2[1] - o12[1]);
    delR2_Right.push_back(delR2[2]);

    if(!relSpace2Scrn(delR1_Left,pitch,yaw,roll,scrPLeft1)){return false;}
    if(!relSpace2Scrn(delR1_Right,pitch,yaw,roll,scrPRight1)){return false;}
    if(!relSpace2Scrn(delR2_Left,pitch,yaw,roll,scrPLeft2)){return false;}
    if(!relSpace2Scrn(delR2_Right,pitch,yaw,roll,scrPRight2)){return false;}

    //printf("scrPLeft1 = (%d,%d) scrPRight1 = (%d,%d)\n",scrPLeft1.X,scrPLeft1.Y,scrPRight1.X,scrPRight1.Y);
    //printf("scrPLeft2 = (%d,%d) scrPRight2 = (%d,%d)\n",scrPLeft2.X,scrPLeft2.Y,scrPRight2.X,scrPRight2.Y);

    return true;
}


void guiUtils::drawTrail(double lat0,double lon0,double alt0,std::vector<double>& vecLatitude_Prev,std::vector<double>& vecLongitude_Prev,std::vector<double>& vecAltitude_Prev,double yaw, double pitch, double roll)
{
    double latP1;
    double lonP1;
    double altP1;

    double latP2;
    double lonP2;
    double altP2;

    bool empty = true;

    Point scrP;
    Point scrPLeft1;
    Point scrPRight1;
    Point scrPLeft2;
    Point scrPRight2;

    std::vector<Point> scrPts;
    std::vector<Point> scrPtsLeft;
    std::vector<Point> scrPtsRight;

    unsigned int trailIdx =  nearestPoint(lat0,lon0,vecLatitude_Prev,vecLongitude_Prev, empty);
    if(empty) return;

    unsigned int indexBound = 0;
    indexBound =  (MAX_TRAIL_LEN < vecLatitude_Prev.size()) ? MAX_TRAIL_LEN:vecLatitude_Prev.size();
	for(unsigned int i = trailIdx ; (i < indexBound + trailIdx) && (indexBound > 1) ; ++i)
	{
        latP1 = vecLatitude_Prev[i];
        lonP1 = vecLongitude_Prev[i];
        altP1 = vecAltitude_Prev[i];

        latP2 = vecLatitude_Prev[i+1];
        lonP2 = vecLongitude_Prev[i+1];
        altP2 = vecAltitude_Prev[i+1];


		if(coordinate2Scr(lat0 ,lon0 ,alt0 + HEIGHT_OF_HEAD ,latP1 ,lonP1 ,altP1 ,pitch ,yaw ,roll ,scrP)){
            //thickLineRGBA(*globgRenderer ,scrP.X ,scrP.Y ,scrP.X ,scrP.Y, 2*LINE_THICKNESS ,200,200,255,255);
            scrPts.push_back(scrP);
        }
        //printf("scrP = (%d,%d)\n",scrP.X,scrP.Y);
            //scrPts.push_back(scrP);

		if(sideLines2Scrn(pitch ,yaw ,roll ,lat0 ,lon0 ,alt0 + HEIGHT_OF_HEAD ,latP1 ,lonP1 ,altP1 ,latP2 ,lonP2 ,altP2 ,scrPLeft1 ,scrPRight1 ,scrPLeft2 ,scrPRight2))
		{
            //thickLineRGBA(*globgRenderer ,scrPLeft1.X ,scrPLeft1.Y ,scrPLeft2.X ,scrPLeft2.Y, LINE_THICKNESS ,100,100,100,250);
            //thickLineRGBA(*globgRenderer ,scrPLeft1.X ,scrPLeft1.Y ,scrPLeft1.X ,scrPLeft1.Y, LINE_THICKNESS ,100,200,200,200);
            //thickLineRGBA(*globgRenderer,scrPRight1.X ,scrPRight1.Y ,scrPRight2.X ,scrPRight2.Y, LINE_THICKNESS ,100,100,100,250);
            //thickLineRGBA(*globgRenderer ,scrPRight1.X ,scrPRight1.Y ,scrPRight1.X ,scrPRight1.Y, LINE_THICKNESS ,100,200,200,200);

            //thickLineRGBA(*globgRenderer ,scrPLeft2.X ,scrPLeft2.Y ,scrPLeft2.X ,scrPLeft2.Y, LINE_THICKNESS ,100,200,200,200);
            //thickLineRGBA(*globgRenderer ,scrPRight2.X ,scrPRight2.Y ,scrPRight2.X ,scrPRight2.Y, LINE_THICKNESS ,100,200,200,200);
            scrPtsLeft.push_back(scrPLeft1);
            scrPtsRight.push_back(scrPRight1);
		}
		//printf("scrPLeft = (%d,%d)\n",scrPLeft1.X,scrPLeft1.Y);
		//printf("scrPRight = (%d,%d)\n",scrPRight1.X,scrPRight1.Y);
    }

    if(scrPts.empty()) return;

    scrPtsLeft.push_back(scrPLeft2);
    scrPtsRight.push_back(scrPRight2);

    for(unsigned int i = 0 ; i < scrPts.size()-1 ; ++i)
    {
        thickLineRGBA(*globgRenderer ,scrPts[i].X ,scrPts[i].Y ,scrPts[i].X ,scrPts[i].Y, 2*LINE_THICKNESS ,200,200,255,255);
        //printf("scrnPts[%d] = (%d,%d)\n",i,scrPts[i].X,scrPts[i].Y);
    }

    for(unsigned int i = 0 ; i < scrPtsLeft.size()-2 && i < scrPtsRight.size()-2 ; ++i)
    {
        thickLineRGBA(*globgRenderer ,scrPtsLeft[i].X ,scrPtsLeft[i].Y ,scrPtsLeft[i+1].X ,scrPtsLeft[i+1].Y, LINE_THICKNESS ,100,100,100,250);
        thickLineRGBA(*globgRenderer ,scrPtsRight[i].X ,scrPtsRight[i].Y ,scrPtsRight[i+1].X ,scrPtsRight[i+1].Y, LINE_THICKNESS ,100,100,100,250);
        thickLineRGBA(*globgRenderer ,scrPtsLeft[i+1].X ,scrPtsLeft[i+1].Y ,scrPtsLeft[i+1].X ,scrPtsLeft[i+1].Y, LINE_THICKNESS ,100,200,200,200);
        thickLineRGBA(*globgRenderer ,scrPtsRight[i+1].X ,scrPtsRight[i+1].Y ,scrPtsRight[i+1].X ,scrPtsRight[i+1].Y, LINE_THICKNESS ,100,200,200,200);

        //printf("scrnPtsLeft[%d] = (%d,%d)\n",i,scrPtsLeft[i].X,scrPtsLeft[i].Y);
        //printf("scrnPtsRight[%d] = (%d,%d)\n",i,scrPtsRight[i].X,scrPtsRight[i].Y);
    }

}

unsigned int guiUtils::nearestPoint(double lat0,double lon0,std::vector<double>& vecLatitude_Prev,std::vector<double>& vecLongitude_Prev, bool& empty)
{
    empty = vecLatitude_Prev.empty();
    if(empty) return 0;
    Coordinate curr(0,0);
    Coordinate nearCurr(0,0);
    gps2linDist(curr,lat0,lon0);

    gps2linDist(nearCurr,vecLatitude_Prev[0],vecLongitude_Prev[0]);
    double d0 = distEuc(curr,nearCurr);
    unsigned int indexMin = 0;


    for(unsigned int i = 1 ; i < vecLatitude_Prev.size() ; ++i)
    {
        gps2linDist(nearCurr,vecLatitude_Prev[i],vecLongitude_Prev[i]);
        double currDistance = distEuc(curr,nearCurr);
        if(currDistance < d0) {
            d0 = currDistance;
            indexMin = i;
        }
    //printf("curr = (%f,%f)\n",curr.X,curr.Y);
    //printf("nearCurr = (%f,%f)\n",nearCurr.X,nearCurr.Y);
    //printf("nearDist = %f\n",d0);
    }
    return indexMin;
}

double guiUtils::distEuc(Coordinate co1 , Coordinate co2)
{
    return sqrt( (co1.X-co2.X)*(co1.X-co2.X) + (co1.Y-co2.Y)*(co1.Y-co2.Y) );
}


void guiUtils::simulationMap(int& counter,double& lat , double& lon , double& alt)
{

    if(counter<=500){
        lat = 35+0.005*cos(PI/250.0*counter);
        lon = 32+0.005*sin(PI/250.0*counter);
        counter++;
    }
    else if(counter<=1000){
        lat = 35 + 0.005*cos(PI-PI/250.0*counter)+0.01;
        lon = 32 + 0.005*sin(PI-PI/250.0*counter);
        counter++;
    }
    else
    counter = 0 ;
}








