#ifndef WIFI_C
#define WIFI_C

int WIFI; //Global variable 
int GPIO, sock, length, n; 
struct sockaddr_in server; 
struct sockaddr_in from; 
uint8_t buf[1024]; 
int argc=3838; //PORT
socklen_t fromlen;

void startWifi(){
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if ( sock < 0 ){
		printf("error Opening Socket");
		//return 0;
	}
	length = sizeof(server);
	bzero(&server, length);
	server.sin_family=AF_INET;
	server.sin_addr.s_addr=INADDR_ANY;
	server.sin_port=htons(atoi("3838")); //PORT
	//server.sin_port=htons(atoi(argv[1])); //default with commen
	if (bind(sock,(struct sockaddr *)&server,length)<0){
		printf("Error binding");
		//return 0;
	}
	
	socklen_t fromlen = sizeof(struct sockaddr_in);
}

void readWifi(){
	 struct timeval tv;
	 tv.tv_sec = 0;
	 tv.tv_usec = 100000;
	 if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
	 	perror("Error");
	}
	n = recvfrom(sock,buf,25,0,(struct sockaddr*)&from,&fromlen);

	//if(buf[0]=='R' && buf[1]=='G'){
	if(buf[9]==0){WIFI=0;printf("Initial\n");}
	if(buf[9]==1){WIFI=3;printf("Ready\n");}
	if(buf[9]==2){WIFI=2;printf("set\n");}
	if(buf[9]==3){WIFI=1;printf("Play\n");}
	if(buf[9]==4){WIFI=0;printf("Finish\n");}
	//}
}

#endif
