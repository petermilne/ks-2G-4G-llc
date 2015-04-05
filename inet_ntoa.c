#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int main(int argc, char* argv[])
{
	if (argc < 2){
		return -1;
	}else if (strcmp(basename(argv[0]), "inet_aton") == 0){
		in_addr_t addr = inet_network(argv[1]);
		printf("0x%08x\n", addr); 
		return addr == -1? 1: 0;
	}else{
		struct in_addr addr;
		if (inet_aton(argv[1], &addr)){
			printf("%s\n", inet_ntoa(addr));
			return 0;
		}else{

		}return 2;
	}
}
