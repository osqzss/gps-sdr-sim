#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>




int sockinit(short port)
{
    int sock = socket(AF_INET,SOCK_STREAM, 0);
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);  
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");  
    if (connect(sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        perror("connect");
        exit(1);
    }


    return sock;
}
void sockclose(int s){
	close(s);
}
void socksend(int s,void *dataa,int siz){
	send(s,dataa,siz,0);
}

long int timem(){
	struct timeval t;
	gettimeofday(&t, NULL);
	return t.tv_sec*1000+t.tv_usec/1000;
}

int udpinit(short port){
    int sock = socket(AF_INET,SOCK_DGRAM, 0);
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);  
    servaddr.sin_addr.s_addr = INADDR_ANY;  
    if (bind(sock, (struct sockaddr *)&servaddr, sizeof(servaddr))<0){
       perror("connect");
       exit(1);
    }
    return sock;
}
int udprecv(int s,void *dataa,int siz){
	struct sockaddr from;
	return recvfrom(s, dataa, siz, 0,&from,sizeof(from));
}

double llhr[3]={39.68,139,76,10};

void threadrecv(){
	printf("listing\n");
	short p=5678;
	int s=udpinit(p);
	while(1){
		udprecv(s,llhr,3*sizeof(double));
		//printf("%lf%lf%lf\n",llhr[0],llhr[1],llhr[2]);
	}
}


		 
