#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.1415926

void rotateSinglePoint(short x, short y, short origin_x, short origin_y, double angle_deg ,short* x_new, short* y_new);

void rotatePts(short x[], short y[], int numPTS, double angle_deg , short origin_x, short origin_y,short x_new[], short y_new[])
{
        for(int i=0; i<numPTS ; ++i)
            rotateSinglePoint(x[i], y[i], origin_x, origin_y, angle_deg , x_new + i, y_new + i);
}



void rotateSinglePoint(short x, short y, short origin_x, short origin_y, double angle_deg ,short* x_new, short* y_new)
{
    double angleRAD = angle_deg/180.0 * PI;
    double x_temp = x-origin_x;
    double y_temp = y-origin_y;

    *x_new = x_temp*cos(angleRAD) + y_temp*sin(angleRAD) + origin_x;
    *y_new = -x_temp*sin(angleRAD) + y_temp*cos(angleRAD) + origin_y;
}
