#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

int main(int argc, char **argv)
{
	struct sockaddr_rc addr = { 0 };
	int s, status, bytes_read;
	char buf[1024] = { 0 };
	char dest[18] = "00:1A:7D:DA:71:13";
	pid_t pid = 0;
	// allocate a socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 22;
	str2ba( dest, &addr.rc_bdaddr );

	// connect to server
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

	if( status < 0 ) perror("uh oh");

	// send a message
	if( status == 0 ) {
		char user_input[1024] = { 0 };
		pid = fork();
		if(pid < 0) perror("Could not fork process!");
		if(pid) while(1) {
			//send a message
			fgets(user_input,1024,stdin);
			strcpy(buf,user_input);
			status = write(s, buf, sizeof(buf));
			if( status < 0 ) perror("uh oh");
		}
		else while(1) {
			//recieve message
			bytes_read = read(s, buf, sizeof(buf));
			if( bytes_read > 0 ) {
				printf("%s", buf);
			}
		}
	}

	if(!pid)
		close(s);

	return 0;
}

