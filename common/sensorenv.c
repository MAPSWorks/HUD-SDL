#include "sensorenv.h"

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;

void* sensors_main(void* arg) {

	VN_ERROR_CODE errorCode;
	Vn200 vn200;
	int i;
	char* buf = (char*)arg;

      	//initialize the sensor, return if not succeeded.
	if(initAsyncSensors(&vn200) == -1) {
		printf("error initializing device\n");
		return 0;
	}

	for (i = 0; i < 10; i++) {

		VnDeviceCompositeData data;

		vn200_getCurrentAsyncData(&vn200, &data);

		sprintf(buf,
			"INS Solution:\n"
			"YPR.Yaw:                %+#7.2f\n"
			"YPR.Pitch:              %+#7.2f\n"
			"YPR.Roll:               %+#7.2f\n",
			data.ypr.yaw,
			data.ypr.pitch,
			data.ypr.roll);

		usleep(500000);
	}
	errorCode = vn200_disconnect(&vn200);

	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		return 0;
	}

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
	else if (errorCode != VNERR_NO_ERROR)
	{
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

