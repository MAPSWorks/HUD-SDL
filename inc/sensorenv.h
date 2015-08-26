#ifndef _SENSORENVS_H_
#define _SENSORENVS_H_

#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"

#define VN_CONNECTION_POLL_RATE	1	// in seconds
#define VN_GET_DATA_ASYNC_RATE 	30000	// in microseconds

#define SENS_REFRESH_RATE 	10000
#define SENS_BUF_SIZE		512

#define VERIFY_CONN_FIX


int initAsyncSensors(Vn200* vn200);
void* sensors_main(void* arg);


#endif /* _SENSORENVS_H_ */

