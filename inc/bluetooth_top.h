#ifndef _BLUETOOH_TOP_H_
#define _BLUETOOH_TOP_H_

#include "common.h"


#define ALEX_NEXUS5_PHONE "2C:54:CF:78:B4:E0"
#define BT_DEST_ADDR	ALEX_NEXUS5_PHONE
#define BT_CHANNEL	6

void* bt_main(void* arg);
int bt_scan();
int bt_connect();
int bt_server();
int bt_client();

typedef struct BT_data {
	char str[BUF_SIZE] = { 0 };
	double rpm = 0, velo = 0;
	int gear = 0;
} BT_data;

#endif // _BLUETOOH_TOP_H_
