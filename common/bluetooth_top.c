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


int BT_main() {
	BT_server();
	return 0;
}

int BT_server() {
	struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
	char wr_buf[1024] = { 0 }, rd_buf[1024] = { 0 };
	int s, client, bytes_read, status;
	pid_t pid;
	socklen_t opt = sizeof(rem_addr);

	printf("Initializing BT_server()\n");

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
	fprintf(stderr, "accepted connection from %s\n", wr_buf);
	memset(wr_buf, 0, sizeof(wr_buf));

	// read data from the client
	pid = fork();
	if(pid < 0) perror("Could not fork process!");
	if(pid) {
		while(1) {
			//send a message
			fgets(wr_buf,1024,stdin);
			status = write(client, wr_buf, sizeof(wr_buf));
			if( status < 0 ) perror("uh oh");
		}
	} else {
		while(1) {
			//recieve message
			bytes_read = read(client, rd_buf, sizeof(rd_buf));
			if( bytes_read > 0 ) {
				printf("%s", rd_buf);
			}
		}
	}

	// close connection
	close(client);
	close(s);
	return 0;
}

int BT_connect() {
	struct sockaddr_rc addr = { 0 };
	int s, status;
	char dest[18] = "00:11:67:C3:CB:EF";

	printf("Attempting connection to address: %s\n",dest);

	// allocate a socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 22;	//	changed to 22
	str2ba( dest, &addr.rc_bdaddr );

	// connect to server
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
	// send a message
	if( status == 0 ) {
		status = write(s, "hello!", 6);
	}

	if( status < 0 ) perror("uh oh");

	close(s);

	return 0;
}

int BT_scan() {
	inquiry_info *ii = NULL;
	int max_rsp, num_rsp;
	int dev_id, sock, len, flags;
	int i;
	char addr[19] = { 0 };
	char name[248] = { 0 };

	dev_id = hci_get_route(NULL);
	sock = hci_open_dev( dev_id );
	if (dev_id < 0 || sock < 0) {
		perror("opening socket");
		exit(1);
	}

	len  = 8;
	max_rsp = 255;
	flags = IREQ_CACHE_FLUSH;
	ii = (inquiry_info*)malloc(max_rsp * sizeof(inquiry_info));

	num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
	if( num_rsp < 0 ) perror("hci_inquiry");

	for (i = 0; i < num_rsp; i++) {
		ba2str(&(ii+i)->bdaddr, addr);
		memset(name, 0, sizeof(name));
		if (hci_read_remote_name(sock, &(ii+i)->bdaddr, sizeof(name), 
					name, 0) < 0)
			strcpy(name, "[unknown]");
		printf("%s  %s\n", addr, name);
	}

	close( sock );
	free(ii);

	return 0;
}

