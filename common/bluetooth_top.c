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
#include "common.h"

extern char bt_buf[BUF_SIZE], gps_buf[BUF_SIZE];
extern bool globQuitSig;

// BT global variables
bool connected;
int client;
char wr_buf[BUF_SIZE] = { 0 }, rd_buf[BUF_SIZE] = { 0 };

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

void* recieve_messages(void* arg) {
	printf("Rcv msgs thread started\n");
	while(connected&&!globQuitSig) {
		int bytes_read = read(client, rd_buf, BUF_SIZE);
		if(bytes_read == -1) {
			sprintf(bt_buf, "Client disconnected");
			connected = 0;
			return 0;
		}
		if(!rd_buf[0] && bytes_read == 16) continue;	//	this is a string that is recieved with every message, should be discarded
		rd_buf[strlen(rd_buf)-1] = '\0';	//	TODO: just for the presentation on 25/5/15
		strcpy(bt_buf,rd_buf);
	}
	return 0;
}

int init_arrays_and_start_threads() {
	printf("Initializing arrays and starting threads\n");
	// create two threads - one for read and one for write
	pthread_t outmsg_th, inmsg_th;

	memset(wr_buf, 0, sizeof(wr_buf));
	memset(rd_buf, 0, sizeof(rd_buf));
	
	pthread_create(&outmsg_th,	NULL,	send_messages,		NULL);
	pthread_create(&inmsg_th,	NULL,	recieve_messages,	NULL);
	
	printf("Threads initialized, waiting for quit sig\n");

	pthread_join(outmsg_th,	NULL);
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
	loc_addr.rc_channel = (uint8_t) 1;
	bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));

	while(!globQuitSig) {
		// put socket into listening mode
		listen(s, 1);

		// accept one connection
		sprintf(bt_buf, "Waiting for connection");

		client = accept(s, (struct sockaddr *)&rem_addr, &opt);
		connected = 1;

		ba2str( &rem_addr.rc_bdaddr, wr_buf );
		sprintf(bt_buf, "accepted connection from %s", wr_buf);
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
	addr.rc_channel = (uint8_t) 1;
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
