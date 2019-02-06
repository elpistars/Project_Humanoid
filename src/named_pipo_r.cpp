#include <iostream> 
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h>
using namespace std;
int main(){
	int fd;
	char * myfifo = "/tmp/myfifo";
	
	//Create named file FIFO
	//mkfifo(<path>,<permission>
	mkfifo(myfifo, 0666);
	struct{
	  double x,y;
        }data;
//	x=0;
	while(1){
		fd=open(myfifo,O_RDONLY);
		read(fd, &data, sizeof(data));
		cout<<data.x<<" | "<<data.y<<endl;
		close(fd);
	}
	return 0;
} 
