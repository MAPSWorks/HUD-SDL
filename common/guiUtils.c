#include "gui.h"
#include <iostream>
#include <vector>

Point::Point(int X, int Y) : X(X), Y(Y) {};

//point functions
void guiUtils::rotate(Point point ,Point& updated_point, Point& origin ,double ang_Deg)
{
	point.X = ( point.X - origin.X )*cos(ang_Deg*180/PI) + ( point.Y - origin.Y )*sin(ang_Deg*180/PI) + origin.X;
	point.Y = -( point.X - origin.X )*sin(ang_Deg*180/PI) + ( point.Y - origin.Y )*cos(ang_Deg*180/PI) + origin.Y;	
}
	
int guiUtils::strech(Point point ,Point& updated_point , Point& origin ,double factor, char xy)
{
	if(xy == 'x') point.X = ( point.X - origin.X )*factor + origin.X;

	else if(xy == 'y') point.X = ( point.Y - origin.Y )*factor + origin.Y;
	
	else{    
		 std::cout << "error in method strech input must be either x or y";
		return -1;
	}
	return 0;
}

int guiUtils::gps2linDist(Point& updated_point ,double lat, double lon)
{
	updated_point.X = Earth_Radius*cos(lat)*cos(lon);
	updated_point.Y = Earth_Radius*cos(lat)*sin(lon);
	return 0;
}




//vector functions
void guiUtils::rotateVec(std::vector<Point> pts ,std::vector<Point>& Updated_pts, Point& origin ,double ang_Deg)
{
	for(unsigned int i = 0 ;  i<pts.size() ; i++)
	{
		rotate(pts[i] ,Updated_pts[i], origin ,ang_Deg);
	}
}
	
int guiUtils::strechVec(std::vector<Point> pts ,std::vector<Point>& Updated_pts, Point& origin ,double factor, char xy)
{
	for(unsigned int i = 0 ;  i<pts.size() ; i++)
	{
		strech(pts[i] ,Updated_pts[i] ,origin ,factor, xy);
	}
	return 0;
}


void guiUtils::norm(short frameWidth, short frameLength ,short framePosX ,short framePosY ,std::vector<Point> pts ,std::vector<Point>& Updated_pts)
{
	//int maxPointX, maxPointY;
	//maxPointX = pts.X.max(); 
	//minPointX = pts.X.min();

	//maxPointY = pts.Y.max(); 
	//minPointY = pts.Y.min();

	//for(unsigned int i = 0 ;  i<pts.size() ; i++)
	//{
	//	Updated_pts[i].X = ( pts[i].X - minPointX )/(maxPointX - minPointX )*frameWidth + framePosX;
	//	Updated_pts[i].Y = ( pts[i].Y - minPointY )/(maxPointY - minPointY )*frameLength + framePosY;	
	//}
}

int guiUtils::gps2linDistVec(std::vector<Point>& Updated_pts ,double lat, double lon)
{
	for(unsigned int i = 0 ;  i<Updated_pts.size() ; i++)
	{
		gps2linDist(Updated_pts[i] ,lat ,lon);
	}
	return 0;
}





