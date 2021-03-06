#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#define BUF_SIZE		2048
#define ALEX_NEXUS5_PHONE	"2C:54:CF:78:B4:E0"
#define FORMULA_BT_ADDR		"98:D3:31:50:1A:BF"
#define ODROID_ADDR		"00:1A:7D:DA:71:13"
#define BEN_PHONE_ADDR		"EC:CB:30:D4:93:1C"

#ifndef BT_DEST_ADDR
#define BT_DEST_ADDR		FORMULA_BT_ADDR
#endif

#ifndef	BT_CHANNEL
#define BT_CHANNEL		7	//	XXX the channel is important and should match the server channel! XXX
#endif

int main(int argc, char **argv)
{
	struct sockaddr_rc addr = { 0 };
	int s, status, bytes_read;
	char wr_buf[BUF_SIZE] = { 0 }, rd_buf[BUF_SIZE] = { 0 };
	char dest[18] = BT_DEST_ADDR;
	pid_t pid = 0;
	// allocate a socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) BT_CHANNEL;
	str2ba( dest, &addr.rc_bdaddr );

	// connect to server
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

	if( status < 0 ) perror("uh oh");

	printf("Connection established with: %s\n", dest);

	// send a message
	if( status == 0 ) {
		pid = fork();
		if(pid < 0) perror("Could not fork process!");
		if(pid) while(1) {
			//send a message
			fgets(wr_buf,BUF_SIZE,stdin);
			status = write(s, wr_buf, sizeof(wr_buf));
			if( status < 0 ) perror("uh oh");
		}
		else while(1) {
			//recieve message
			bytes_read = read(s, rd_buf, sizeof(rd_buf));
			bytes_read < BUF_SIZE ? (rd_buf[bytes_read] = 0) : (rd_buf[BUF_SIZE-1] = 0);	//	terminating character
			fflush(stdout);
			printf("%s\n", rd_buf);
		}
	}

	if(!pid)
		close(s);

	return 0;
}

