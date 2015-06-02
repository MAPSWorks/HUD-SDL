#include "sensorenv.h"
#include "common.h"

extern char sensors_buf[BUF_SIZE], gps_buf[BUF_SIZE];
extern VnDeviceCompositeData sensorData;

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;

void* sensors_main(void* arg) {

	VN_ERROR_CODE errorCode;
	Vn200 vn200;

	printf("Initializing sensornav\n");

      	//initialize the sensor, return if not succeeded.
	if(initAsyncSensors(&vn200) == -1) {
		sprintf(gps_buf,
			"GPS.Lat: %+#7.2f, "
			"GPS.Lon: %+#7.2f, "
			"GPS.Alt: %+#7.2f\n",
			1.23,
			4.56,
			7.89);

		sprintf(sensors_buf,
			"YPR.Yaw: %+#7.2f, "
			"YPR.Pitch: %+#7.2f, "
			"YPR.Roll: %+#7.2f",
			1.23,
			4.56,
			7.89);

		return 0;
	}

	while (1) {

		vn200_getCurrentAsyncData(&vn200, &sensorData);

		sprintf(gps_buf,
			"GPS.Lat: %+#7.2f, "
			"GPS.Lon: %+#7.2f, "
			"GPS.Alt: %+#7.2f\n",
			sensorData.gpsPosLla.c0,
			sensorData.gpsPosLla.c1,
			sensorData.gpsPosLla.c2);

		sprintf(sensors_buf,
			"YPR.Yaw: %+#7.2f, "
			"YPR.Pitch: %+#7.2f, "
			"YPR.Roll: %+#7.2f",
			sensorData.ypr.yaw,
			sensorData.ypr.pitch,
			sensorData.ypr.roll);

		//usleep(SENS_REFRESH_RATE);
	}

	errorCode = vn200_disconnect(&vn200);

	if (errorCode != VNERR_NO_ERROR)
		perror("Error encountered when trying to disconnect from the sensor.\n");

	return 0;
}

int initAsyncSensors(Vn200* vn200) 
{
	VN_ERROR_CODE errorCode;
	errorCode = vn200_connect(vn200,COM_PORT,BAUD_RATE);
	
	//the struct VnDeviceCompositeData in vndevice.h contain all posible variables.

	/* Make sure the user has permission to use the COM port. */
	if (errorCode == VNERR_PERMISSION_DENIED) {
		printf("Current user does not have permission to open the COM port.\n");
		printf("Try running again using 'sudo'.\n");

		return -1;
	}
	else if (errorCode != VNERR_NO_ERROR) {
		printf("Error encountered when trying to connect to the sensor.\n");
		return -1;
	}
	
	/* Configure the VN-200 to output asynchronous data. */
	errorCode = vn200_setAsynchronousDataOutputType(vn200,VNASYNC_VNINS,true);

	/* Pause to ensure we have received the first asynchronous data record
	   from the sensor. */
	sleep(1);
	return 1;
}

