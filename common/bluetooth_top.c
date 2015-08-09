#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/sdp.h>
#include <bluetooth/sdp_lib.h>
#include <bluetooth/rfcomm.h>
#include "bluetooth_top.h"
#include <errno.h>
#include <wait.h>
#include <pthread.h>

extern char gps_buf[BUF_SIZE];
extern bool globQuitSig;

// BT global variables
bool connected;
int client;
char wr_buf[BUF_SIZE] = { 0 }, rd_buf[BUF_SIZE] = { 0 };
BT_data bt_data;

void* bt_main(void* arg) {
	bt_server();
	return 0;
}

void* send_messages(void* arg) {
	printf("Send msgs thread started\n");
	while(connected&&!globQuitSig) {
		strcpy(wr_buf,gps_buf);
		if( write(client, wr_buf, BUF_SIZE) < 0 ) perror("uh oh");
		usleep(BT_REFRESH_RATE);
	}
	return 0;
}

static void parse_msg(BT_data* m) {
	char* str = m->str;
	while(*str && *(str++) != 'r');
	sscanf(str, "%lf", &m->rpm);
	while(*str && *(str++) != 'v');
	sscanf(str, "%lf", &m->velo);
	while(*str && *(str++) != 'g');
	sscanf(str, "%d", &m->gear);
}

static void copy_bt_data(BT_data* to, BT_data* from) {
	strcpy(to->str, from->str);
	to->rpm = from->rpm;
	to->velo = from->velo;
	to->gear = from->gear;
}

void* recieve_messages(void* arg) {
	printf("Rcv msgs thread started\n");
	while(connected&&!globQuitSig) {
#ifndef BT_INPUT_FROM_STDIN
		int bytes_read = read(client, rd_buf, BUF_SIZE);
#else
		int bytes_read = scanf("%s", rd_buf);
#endif
		rd_buf[BUF_SIZE-1] = '\0';	// to prevent segfault
		BT_data incoming;
		if(bytes_read == -1) {
			sprintf(bt_data.str, "Client disconnected");
			connected = 0;
			return 0;
		}
		if(!rd_buf[0] && bytes_read == 16) continue;	//	this is a string that is recieved with every message, should be discarded
		strcpy(incoming.str, rd_buf);
		parse_msg(&incoming);
		copy_bt_data(&bt_data, &incoming);
	}
	return 0;
}

int init_arrays_and_start_threads() {
	printf("Initializing arrays and starting threads\n");
	// create two threads - one for read and one for write
	//pthread_t outmsg_th
	pthread_t inmsg_th;

	memset(wr_buf, 0, sizeof(wr_buf));
	memset(rd_buf, 0, sizeof(rd_buf));
	
	//pthread_create(&outmsg_th,	NULL,	send_messages,		NULL);
	pthread_create(&inmsg_th,	NULL,	recieve_messages,	NULL);
	
	printf("Threads initialized, waiting for quit sig\n");

	//pthread_join(outmsg_th,	NULL);
	pthread_join(inmsg_th,	NULL);

	printf("Exitintg init_arrays_and_start_threads\n");

	return 0;
}

int bt_server() {
	struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
	int s;
	socklen_t opt = sizeof(rem_addr);

	printf("Initializing bt_server\n");

	// allocate socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// bind socket to port 1 of the first available 
	// local bluetooth adapter
	loc_addr.rc_family = AF_BLUETOOTH;
	loc_addr.rc_bdaddr = {{0,0,0,0,0,0}}; 
	loc_addr.rc_channel = (uint8_t) BT_CHANNEL;
	bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));

	while(!globQuitSig) {
		// put socket into listening mode
		listen(s, 1);

		// accept one connection
		sprintf(bt_data.str, "Waiting for connection");

#ifndef BT_INPUT_FROM_STDIN
		client = accept(s, (struct sockaddr *)&rem_addr, &opt);
#endif
		connected = 1;

		ba2str( &rem_addr.rc_bdaddr, wr_buf );
		sprintf(bt_data.str, "accepted connection from %s", wr_buf);
		init_arrays_and_start_threads();
	}

	// close connection
	close(client);
	close(s);
	return 0;
}

int bt_client() {
	struct sockaddr_rc addr = { 0 };
	int s, status;
	char dest[18] = BT_DEST_ADDR;

	// allocate a socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	printf("Socket allocated\n");

	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) BT_CHANNEL;
	str2ba( dest, &addr.rc_bdaddr );

	while(!globQuitSig) {
		// connect to server
		printf("Trying to connect to addr: %s\n", dest);
		status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

		if( !status ) {
			connected = 1;
			init_arrays_and_start_threads();
		} else {
			perror("uh oh");
			break;
		}
	}

	close(s);
	return 0;
}
