#ifndef _SENSORENVS_H_
#define _SENSORENVS_H_

#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"

int initAsyncSensors(Vn200* vn200);
void* sensors_main(void* arg);


#endif /* _SENSORENVS_H_ */

