#include "sensorenv.h"
#include "common.h"

extern bool globQuitSig;

char sensors_buf[SENS_BUF_SIZE], gps_buf[SENS_BUF_SIZE] ,velocity_buf[SENS_BUF_SIZE];
VnDeviceCompositeData sensorData;

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "/dev/ttyUSB0";
const int BAUD_RATE = 115200;

void* sensors_main(void* arg) {
	Vn200 vn200;

	while (!globQuitSig) {
		while(initAsyncSensors(&vn200) == -1)	//	in case we failed sleep and try to reconnect
			sleep(VN_CONNECTION_POLL_RATE);
		printf("Sensornav connected succesfully\n");
		while(vn200_verifyConnectivity(&vn200)) {
			usleep(VN_GET_DATA_ASYNC_RATE);
			vn200_getCurrentAsyncData(&vn200, &sensorData);

			sprintf(gps_buf,
				"GPS.Lat: %+#7.2f, "
				"GPS.Lon: %+#7.2f, "
				"GPS.Alt: %+#7.2f\n",
				sensorData.latitudeLongitudeAltitude.c0,
				sensorData.latitudeLongitudeAltitude.c1,
				sensorData.latitudeLongitudeAltitude.c2);

			sprintf(sensors_buf,
				"YPR.Yaw: %+#7.2f, "
				"YPR.Pitch: %+#7.2f, "
				"YPR.Roll: %+#7.2f",
				sensorData.ypr.yaw,
				sensorData.ypr.pitch,
				sensorData.ypr.roll);

			sprintf(velocity_buf,
				"VelocityX: %+#7.2f, "
				"VelocityY: %+#7.2f, "
				"VelocityZ: %+#7.2f",
				sensorData.velocity.c0,
				sensorData.velocity.c1,
				sensorData.velocity.c2);
		}
		printf("Disconnecting from sensors\n");
		if (vn200_disconnect(&vn200) != VNERR_NO_ERROR) 
			perror("Error encountered when trying to disconnect from the sensor\n");
	}

	return 0;
}

int initAsyncSensors(Vn200* vn200)
{
	VN_ERROR_CODE errorCode;
	errorCode = vn200_connect(vn200,COM_PORT,BAUD_RATE);

	if (errorCode == VNERR_PERMISSION_DENIED) {
		printf("Current user does not have permission to open the COM port\n");
		printf("Try running again using 'sudo'.\n");
		return -1;
	}
	else if (errorCode != VNERR_NO_ERROR) {
		printf("Error encountered when trying to connect to the sensor\n");
		return -1;
	}
	errorCode = vn200_setAsynchronousDataOutputType(vn200,VNASYNC_VNINS,true);
	sleep(1);
	return 1;
}
