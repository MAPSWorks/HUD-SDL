#include "gui.h"
#include <iostream>
#include <vector>
#include <math.h>



Point::Point(int X, int Y) : X(X), Y(Y) {};

//point functions
void guiUtils::rotatePoint(Point& point ,Point& updated_point, Point& origin ,double ang_Deg)
{
	updated_point.X = ( point.X - origin.X )*cos(ang_Deg*PI/180) + ( point.Y - origin.Y )*sin(ang_Deg*PI/180) + origin.X;
	updated_point.Y = -( point.X - origin.X )*sin(ang_Deg*PI/180) + ( point.Y - origin.Y )*cos(ang_Deg*PI/180) + origin.Y;
}

void guiUtils::strech(Point& point ,Point& updated_point , Point& origin ,double factor, char xy)
{
	if(xy == 'x') point.X = ( point.X - origin.X )*factor + origin.X;

	else if(xy == 'y') point.X = ( point.Y - origin.Y )*factor + origin.Y;

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
		Updated_pts[i].X = ( pts[i].X - minPointX )/(maxPointX - minPointX )*MAP_FRAME_WIDTH + MAP_FRAME_POS_X;
		Updated_pts[i].Y = ( pts[i].Y - minPointY )/(maxPointY - minPointY )*MAP_FRAME_LENGTH + MAP_FRAME_POS_Y;
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

//bool guiUtils::isPontInFrame(Point point)
//{
//   if
//}



