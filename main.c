#include "common.h"
#include "gui.h"
#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"
#include "sensorenv.h"
#include "bluetooth_top.h"
#include <pthread.h>
#include <signal.h>

char gps_buf[BUF_SIZE] = { 0 }, sensors_buf[BUF_SIZE] = { 0 }, bt_buf[BUF_SIZE] = { 0 } ,velocity_buf[BUF_SIZE] = { 0 };
bool globQuitSig = false;

VnDeviceCompositeData sensorData;

void* globQuitSigHandler(void* bt_thread){
	while(!globQuitSig);
	pthread_kill(*(pthread_t*)bt_thread, SIGKILL);
	return 0;
}

int main()
{
	pthread_t sensors_thread, bt_thread, gui_thread, globQuitSigHandler_thread;

	pthread_create(&sensors_thread,	NULL,	sensors_main,	NULL);
	pthread_create(&bt_thread,	NULL,	bt_main,	NULL);
	pthread_create(&gui_thread,	NULL,	gui_main,	NULL);
	pthread_create(&globQuitSigHandler_thread,	NULL,	globQuitSigHandler,	&bt_thread);

	pthread_join(globQuitSigHandler_thread,		NULL);
	pthread_join(gui_thread,	NULL);
	pthread_join(sensors_thread,	NULL);
	pthread_join(bt_thread,		NULL);

	return 0;
}

