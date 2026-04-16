#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <iostream>
#include <errno.h> 
#include "easylogging++.h"
#include "../firmware/protocol.h"
#include "update_opencam3d.h"

// init easylogging
INITIALIZE_EASYLOGGINGPP

int main(int argc, char *argv[])
{
	int server_sock = DF_FAILED;
	Init_TCP(server_sock);

	while(1) 
	{
		if (Interact_TCP(server_sock) != DF_SUCCESS) {	return -1; }
	}

    close(server_sock);

	return 0;
}