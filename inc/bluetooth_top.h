#ifndef _BLUETOOH_TOP_H_
#define _BLUETOOH_TOP_H_

#define BT_DEST_ADDR	"98:D3:31:50:1A:B6"

void* bt_main(void* arg);
int bt_scan();
int bt_connect();
int bt_server();
int bt_client();

#endif // _BLUETOOH_TOP_H_
