#include "slipAndGear.h"

#define SLIP_TRESHOLD 5
#define PIE 3.14159265 //like the cake :)

//values for the Optimal Shift Point taken from the table below

#define OSP1 8387.29
#define OSP2 7911.1
#define OSP3 7623.54
#define OSP4 7694.66
#define OSP5 7562.64	 

//this value was taken arbitrary it change significantly between cars
#define OPS_DOWN 2500

/**************************************
slip formula explenationL:
we assume that no lateral slip occured.
so we have the vectoric eq:
V(wheels) + V(sideslip) = V(gps)
V(wheels) vertical to V(sidesleep)
therefore: cos(teta) = V(wheels)/V(gps)
*************************************/

double slipApproximation (double wheelVelocity,double gpsVelocity)
{
	if ((gpsVelocity - wheelVelocity) > SLIP_TRESHOLD)
		{
			return (gpsVelocity - wheelVelocity);
		}
	else
		return 0;
}

double slipAngle (double wheelVelocity,double gpsVelocity)
{
	if ((gpsVelocity - wheelVelocity) > SLIP_TRESHOLD)
		{
			return (acos(gpsVelocity/wheelVelocity)*180.0/PIE);
		}
	else
		return 0;
}


/***********************************************
this recomendation of the shift point taken from the internet after a bit of research.
the recomendation does NOT fit to our formula, it was taken from other formula cars
however, to take the best of it tests should have done on the current formula.
the optimal shift point table:
GEAR                            1st	2nd	3rd	4th	5th	6th
Gear Ratio			3.827	2.36	1.685	1.312	1	0.793
Optimal Shift Point (RPM)	8387.29	7911.1	7623.54	7694.66	7562.64	
RPM at next gear		5172.2	5648.39	5935.95	5864.83	5997.03

this fucntion return	-1 for recomendation to shift down the gear
			+1 to shift up
			0 todo nothing
and by the way Eden is gay :)
************************************************/
int gearRecomendation(double rpm, int gear)
{
	if (rpm < OPS_DOWN && (gear > 1))
		return -1;
	if (gear == 1 && rpm > OSP1 )
		return 1;
	if (gear == 2 && rpm > OSP2)
		return 1;
	if (gear == 3 && rpm > OSP3)
		return 1;
	if (gear == 4 && rpm > OSP4)
		return 1;
	if (gear == 5 && rpm > OSP5)
		return 1;
	return 0;
}

