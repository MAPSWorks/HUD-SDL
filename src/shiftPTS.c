#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void shiftPTS(short x[], short y[], int numPTS, short delX, short delY,short x_next[], short y_next[])
{
    for(int i=0 ; i<numPTS ; ++i)
    {
        x_next[i] = x[i] + delX;
        y_next[i] = y[i] + delY;
    }
}
