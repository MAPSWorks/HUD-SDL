#ifndef SLIP_AND_GEAR_H_
#define SLIP_AND_GEAR_H_

#include <cmath>
#include <stdlib.h>

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

double slipApproximation (double wheelVelocity,double gpsVelocity);

double slipAngle (double wheelVelocity,double gpsVelocity);

int gearRecomendation(double rpm, int gear);

#endif
