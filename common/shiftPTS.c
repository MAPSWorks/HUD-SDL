#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void shitPTS(short x[], short y[], int numPTS, short delX, short delY)
{
    for(int i=0 ; i<numPTS ; ++i)
    {
        x[i] = x[i] + delX;
        y[i] = y[i] + delY;
    }
}
