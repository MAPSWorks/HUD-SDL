#ifndef _BLUETOOH_TOP_H_
#define _BLUETOOH_TOP_H_

#include "common.h"

#define BT_REFRESH_RATE 	1000000

#define BT_BUF_SIZE 		2048	//	it appears a minimum of 1024 bytes buffer size is required, 2048 is taken for stability

#define ALEX_NEXUS5_PHONE 	"2C:54:CF:78:B4:E0"
#define FORMULA_BT_ADDR		"98:D3:31:50:1A:BF"
#define BEN_PHONE_ADDR		"EC:CB:30:D4:93:1C"
#define SHULMANS_ARDUINO_ADDR 	"20:15:02:28:17:62"

#ifndef BT_DEST_ADDR
#define BT_DEST_ADDR	FORMULA_BT_ADDR
#endif

#ifndef BT_CHANNEL
#define BT_CHANNEL	1
#endif

void* bt_main(void* arg);
int bt_scan();
int bt_connect();
int bt_server();
int bt_client();

typedef struct BT_data {
	char str[BT_BUF_SIZE] = { 0 };
	double rpm = 0, velo = 0;
	int gear = 0;
} BT_data;

#endif // _BLUETOOH_TOP_H_
