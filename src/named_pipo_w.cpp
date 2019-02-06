#include <iostream> 
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h> 
using namespace std;
int main() 
{ 
    int fd; 
  
    // FIFO file path 
    char * myfifo = "/tmp/myfifo"; 
  
    // Creating the named file(FIFO) 
    // mkfifo(<pathname>, <permission>) 
    mkfifo(myfifo, 0666); 
  
    int x=1; 
    while (x<100) 
    { 
        // Open FIFO for write only 
        fd = open(myfifo, O_WRONLY); 
	//write fifo
	cout<<"x ="<<x<<endl;
        write(fd, &x, sizeof(x));  
        close(fd);
	x++; 
	sleep(0.5);
    } 
    return 0; 
} 
