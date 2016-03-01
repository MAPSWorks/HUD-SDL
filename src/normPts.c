#include "gui.h"
#include <stdio.h>
#include <stdlib.h>

//Description - This functions normalizes a set of points to fit a frame

//frame is an int array of size 4 which describes the corners of the map.
//frame = {maxX , minX ,maxY ,min Y}

int frameXY(double GPS_PTS_X[] ,double GPS_PTS_Y[] ,int numPTS ,double GPS_frame[]);

int normPts(double GPS_PTS_X[] ,double GPS_PTS_Y[] ,int numPTS, short frame[] , short MAP_PTS_X[] , short MAP_PTS_Y[])
{
    double GPS_frame[4] = { 0 };

    if( frameXY(GPS_PTS_X ,GPS_PTS_Y ,numPTS ,GPS_frame) == 0 )
    {
        for(int i=0 ; i<numPTS ; ++i)
        {
        //ptsX
            if(GPS_frame[1] < GPS_frame[0]) //Avoid Div by 0
             MAP_PTS_X[i] = ( GPS_PTS_X[i] - GPS_frame[1] ) / ( GPS_frame[0] - GPS_frame[1] ) * ( frame[0]-frame[1] ) + frame[1];
            else{printf("error in normPTS Div by 0"); return -1;}
        //ptsY
            if(GPS_frame[2] < GPS_frame[3]) //Avoid Div by 0
             MAP_PTS_Y[i] = ( GPS_PTS_Y[i] - GPS_frame[3] ) / ( GPS_frame[2] - GPS_frame[3] ) * ( frame[2]-frame[3] ) + frame[3];
            else{printf("error in normPTS Div by 0"); return -1;}

        }
    }
    else{printf("error in normPTS"); return -1;}
    return 0;
}

int frameXY(double GPS_PTS_X[] ,double GPS_PTS_Y[] ,int numPTS ,double GPS_frame[])
{
    if(GPS_frame == NULL) {
        printf("error in frameXY no mem aloc");
        return -1;
    }
//Random initialization
    double maxX = GPS_PTS_X[0];
    double maxY = GPS_PTS_Y[0];
    double minX = GPS_PTS_Y[0];
    double minY = GPS_PTS_Y[0];
//find corners
    for(int i=1 ; i<numPTS ; i++)
    {
         maxX = (maxX > GPS_PTS_X[i]) ? maxX:GPS_PTS_X[i];
         minX = (minX < GPS_PTS_X[i]) ? minX:GPS_PTS_X[i];
         maxY = (maxY > GPS_PTS_Y[i]) ? maxX:GPS_PTS_Y[i];
         minY = (minY < GPS_PTS_Y[i]) ? minX:GPS_PTS_Y[i];

    }
    GPS_frame[0] = maxX;
    GPS_frame[1] = minX;
    GPS_frame[2] = maxY;
    GPS_frame[3] = minY;
    return 0;
}
