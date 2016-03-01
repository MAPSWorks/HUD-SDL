#include <stdio.h>
#include <math.h>

//Haversine - geodesic distance between two points
//WGS84 - World Geodesic System 1984
#define Earth_Radius 6372.797560856
#define PI 3.14159265359

//This function converts Longitude , Latitue in angles to linear distance (meters)

int gps2linearDist(double lat, double lon ,short XYcoordiates[])
{
    XYcoordiates[0] = Earth_Radius*cos(lat)*cos(lon);
    XYcoordiates[1] = Earth_Radius*cos(lat)*sin(lon);
    return 0;
}

