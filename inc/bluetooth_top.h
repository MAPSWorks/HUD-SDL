#ifndef _BLUETOOH_TOP_H_
#define _BLUETOOH_TOP_H_

#define ALEX_NEXUS5_PHONE "2C:54:CF:78:B4:E0"
#define BT_DEST_ADDR	 ALEX_NEXUS5_PHONE

void* bt_main(void* arg);
int bt_scan();
int bt_connect();
int bt_server();
int bt_client();

#endif // _BLUETOOH_TOP_H_
