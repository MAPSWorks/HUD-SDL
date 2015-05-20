#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"
#include "sensorenv.h"
#include "bluetooth_top.h"
#include <pthread.h>

struct buffs {
	char* sensors_buf;
	char* bt_buf;
};

static int pretty_print(struct buffs* buffers) {
	//puts(buffers->sensors_buf);
	//puts(buffers->bt_buf);
	return 0;
}

// This is a test comment
int main()
{
	pthread_t sensors_thread, bt_thread;
	char sensors_buf[1024];
	char bt_buf[1024];
	struct buffs buffers = { sensors_buf, bt_buf };

	pthread_create(&sensors_thread,	NULL,	sensors_main,	sensors_buf);
	pthread_create(&bt_thread,	NULL,	bt_main,	bt_buf);

	while(1) {
		pretty_print(&buffers);
		sleep(1);
	}

	pthread_join(sensors_thread,	NULL);
	pthread_join(bt_thread,		NULL);

	return 0;
}

