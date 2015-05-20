#ifndef _BLUETOOH_TOP_H_
#define _BLUETOOH_TOP_H_

#define BUF_SIZE 1024

void* bt_main(void* arg);
int bt_scan();
int bt_connect();
int bt_server(char* globuf);

#endif // _BLUETOOH_TOP_H_
