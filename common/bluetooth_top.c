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


void* bt_main(void* arg) {
	bt_server((char*)arg);
	return 0;
}

struct arg {
	char* buf;
	char* globuf;
	int client;
};

void* send_messages(void* arg) {
	struct arg* params = (struct arg*)arg;
	char* wr_buf = 	params->buf;
	int client   = 	params->client;
	int status = 0;

	while(1) {
		fgets(wr_buf,BUF_SIZE,stdin);
		status = write(client, wr_buf, BUF_SIZE);
		if( status < 0 ) perror("uh oh");
	}
}

void* recieve_messages(void* arg) {
	struct arg* params = (struct arg*)arg;
	char* globuf =	params->globuf;
	char* rd_buf = 	params->buf;
	int client   = 	params->client;

	while(1) {
		int bytes_read = 0;
		bytes_read = read(client, rd_buf, BUF_SIZE);
		if(!rd_buf[0] && bytes_read == 16) continue;
		strcpy(globuf,rd_buf);
	}
}

int bt_server(char* globuf) {
	struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
	char wr_buf[BUF_SIZE] = { 0 }, rd_buf[BUF_SIZE] = { 0 };
	int s, client;
	socklen_t opt = sizeof(rem_addr);

	printf("Initializing bt_server\n");

	// allocate socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// bind socket to port 1 of the first available 
	// local bluetooth adapter
	loc_addr.rc_family = AF_BLUETOOTH;
	loc_addr.rc_bdaddr = *BDADDR_ANY;
	loc_addr.rc_channel = (uint8_t) 22;	//	changed to channel 22
	bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));

	// put socket into listening mode
	listen(s, 1);

	// accept one connection
	printf("Waiting for connection\n");
	client = accept(s, (struct sockaddr *)&rem_addr, &opt);

	ba2str( &rem_addr.rc_bdaddr, wr_buf );
	sprintf(globuf, "accepted connection from %s\n", wr_buf);
	memset(wr_buf, 0, sizeof(wr_buf));

	// create two threads - one for read and one for write
	pthread_t outmsg_th, inmsg_th;
	struct arg outmsg_args 	= { wr_buf, globuf, client };
	struct arg inmsg_args 	= { rd_buf, globuf, client };

	pthread_create(&outmsg_th,	NULL,	send_messages,		&outmsg_args);
	pthread_create(&inmsg_th,	NULL,	recieve_messages,	&inmsg_args);

	pthread_join(outmsg_th,	NULL);
	pthread_join(inmsg_th,	NULL);

	// close connection
	close(client);
	close(s);
	return 0;
}

