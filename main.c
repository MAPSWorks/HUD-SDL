#include "common.h"
#include "gui.h"
#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"
#include "sensorenv.h"
#include "bluetooth_top.h"
#include <pthread.h>

char gps_buf[BUF_SIZE] = { 0 }, sensors_buf[BUF_SIZE] = { 0 }, bt_buf[BUF_SIZE] = { 0 } ,velocity_buf[BUF_SIZE] = { 0 };
VnDeviceCompositeData sensorData;

// This is a test comment
int main()
{
	pthread_t sensors_thread, bt_thread, gui_thread;

	pthread_create(&sensors_thread,	NULL,	sensors_main,	NULL);
	pthread_create(&bt_thread,	NULL,	bt_main,	NULL);
	pthread_create(&gui_thread,	NULL,	gui_main,	NULL);

	pthread_join(gui_thread,	NULL);
	pthread_join(sensors_thread,	NULL);
	pthread_join(bt_thread,		NULL);

	return 0;
}

