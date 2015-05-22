#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"
#include "sensorenv.h"
#include "bluetooth_top.h"
#include <pthread.h>

#define REFRESH_RATE 100000

static struct buffs {
	char* sensors_buf;
	char* bt_buf;
};

static int pretty_print(struct buffs* buffers) {

	puts("###################################");

	// print sensor buffer
	fputs(buffers->sensors_buf,stdout);

	// print bt buffer
	if(buffers->bt_buf[0])
		fputs(buffers->bt_buf,stdout);
	else
		fputs("Bluetooth buffer empty\n",stdout);

	puts("###################################");

	return 0;
}

// This is a test comment
int main()
{
	pthread_t sensors_thread, bt_thread;
	char sensors_buf[1024] = { 0 }, bt_buf[1024] = { 0 };
	struct buffs buffers = { sensors_buf, bt_buf };

	pthread_create(&sensors_thread,	NULL,	sensors_main,	sensors_buf);
	pthread_create(&bt_thread,	NULL,	bt_main,	bt_buf);

	while(1) {
		pretty_print(&buffers);
		usleep(REFRESH_RATE);
	}

	pthread_join(sensors_thread,	NULL);
	pthread_join(bt_thread,		NULL);

	return 0;
}

