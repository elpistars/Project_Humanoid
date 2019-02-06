#include <iostream>
#include <sys/types.h>
#include <sys/socket.h> 
#include <netinet/in.h>
#include <netdb.h>
#include <sstream>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <cstring>
#include <string>
#include <sys/time.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <termio.h>
#include <termios.h>
using namespace std;
//using namespace cv;
char lihat[10];
int b=0;

int main()
{

        int c,status, pid, pipefds[2],tanda;
        char instring[10];
//	int instring;
	/* Create the pipe and return 2 file descriptors in the pipefds array */
	/* This operation is done before the fork so that both processes will */
	/* know about the same pipe, which will allow them to communicate.  */
        status = pipe(pipefds); 
	if (status == -1)
	{
		perror("Trouble");
		exit(1);
	}

	/* create child process; both processes continue from here */
        pid = fork();
	int naon=0;
	int a;
while(1){
        if (pid == -1)           
	{
		perror("Trouble");
		exit(2);
	}
        else if (pid == 0)           /* child : sends message to parent*/
	{
		/* close unused end of pipe */
		/* because this process only needs to write */
		close(pipefds[0]);

		/* send 7 characters in the string, including end-of-string */
//	        cout<<naon<<endl;
//		memset(lihat,0,1);
		if(naon==1){
//		lihat[1]='1';
		memset(lihat,'1',1);
//		b=1;
		naon=0;}
		else if(naon==0){
//		lihat[1]='0';
  		memset(lihat,'0',1);
//		b=0;
		naon=1;}
//		cout<<"naon : "<<naon<<endl;
//		cout<<"pipefds[1] : "<<pipefds[1]<<endl;
		write(pipefds[1],lihat, 1);
		cout<<"lihat ="<<lihat<<endl;
		sleep(1);
	//	close(pipefds[1]);
	//	exit(0);
	}
        else		/* parent : receives message from child */
	{
		/* close unused end of pipe */
		/* because this process only needs to read */
		close(pipefds[1]);
//		memset(instring,'0',1);
		/* read from the pipe */
	    //instring = '1';
		read(pipefds[0],instring, 1);
		cout << "instring = "<<instring<<endl;
//		cout<<"pipefds[0] : "<<pipefds[0]<<endl;
		tanda = *instring-'0';
		cout<<"tanda : "<<tanda<<endl;
		sleep(1);
//		cout << "Just received a message that says: " <<instring << endl;
//		if(a==2){
//		a=0;}
//		a++;
//		tanda = instring[0] - '0';
//		cout << "a = "<<a<<endl;
	//	close(pipefds[0]);
//	memset(instring,0,1);
	//	exit(0);
	}
//	if(lihat=='1'){instring=1;}
//	else if(lihat=='0'){instring=0;}
//	cout<<"a = "<<a<<endl;
//	c=b;
//	if(c==0){a=0;}
//	else if(c==1){a=1;}
//	close(pipefds[1]);
//	read(pipefds[0], instring, 1);
//	*instring = *lihat;
//	cout << "instrng = "<<instring<<endl;
//	if(*instring=='1'){a=1;}else if(*instring=='0'){a=0;}
//	sleep(1);
//	close(pipefds[0]);
}
}
