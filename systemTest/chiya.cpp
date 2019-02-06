#ifndef CHIYA_C
#define CHIYA_C

#include "/home/pi/systemTest/ayachan.cpp"
#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"
#include "/home/pi/systemTest/searchimage.cpp"
//#include "/home/pi/systemTest/pointImage.cpp"

char lihat[10];
//char const *lihat;
char compro[10];
int comsen=0;
int comnentu=0;

pthread_mutex_t lock;
pthread_t thread1,thread2,thread3,thread4,thread5;

//void *geraklurus(void *arg){
void geraklurus(){
com=0;
switch(comnentu){
	case 0:
		berdiri();
		break;
	case 1:
		switch(comger){
			case 1:
				miring();
				break;
			case 2:
				miring2();
				break;
			case 3:
				miring3();
				break;
		}
	case 2:
		switch(comkir){
			case 1:
				kiriputar1();
				break;
			case 2:
				kiriputar2();
				break;
			}	
	case 3:
		switch(comkan){
			case 1:
				kananputar1();
				break;
			case 2:
				kananputar2();
				break;
			}
}
			comser=1;
}

int main()
{

    int result, status, pidpro, pipefds[2];
    comger=0;
	com=1;
	int i=0;
    status = pipe(pipefds); 
	if (status == -1)
	{
		perror("Trouble");
		exit(1);
	}

	
//=================Dynamixel========================	
    result = dxl_initialize(0,1);
    if(result == 1){
        printf("Sukses buka koneksi :)");}
	else{
        printf("Gagal buka koneksi :(");
    }
	berdiri();
						
//=================ImageProc====================	
	aimage=512;
	bimage=450;
	dxl_write_word(20,30,aimage); // kamera ke posisi tengah
	dxl_write_word(19,30,bimage);
 	cout<<" abc "<<endl;
	if(!cap.isOpened()) // kondisi dimana jika kamera tidak terdeteksi maka program akan tertutup
     return -1;
//	cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
//    	cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);

	dxl_write_word(19,32,1023); // set speed servo 
	dxl_write_word(20,32,1023);
    error_sblmDximage=0;
    error_sblmDyimage=0;
//======================================================
        
	pidpro = fork();
while(1){    
	
//	ge = 0; 
   if (pidpro == -1)
	{
		perror("Trouble");
		exit(2);
	}
        else if (pidpro == 0)
	{
		close(pipefds[0]);

		melihat();

			switch(comimage){
			case 0:
				memset(lihat,'0',1);
				break;
			case 1:
				memset(lihat,'1',1);
				break;
			case 2:
				memset(lihat,'2',1);
				break;
			case 3:
				memset(lihat,'3',1);
				break;
		}
		write(pipefds[1],lihat,1);


		cout<<"aimage : "<<aimage<<" bimage : "<<bimage<<endl;
	}
        else
	{
		close(pipefds[1]);
		read(pipefds[0],compro,1);
		ge = *compro-'0';
		cout<<"ge : "<<ge<<endl;
		if(ge==0){comnentu=0;}
		else if(ge==1){comnentu=1;}
		else if(ge==2){comnentu=2;}
		else if(ge==3){comnentu=3;}
		//comger=0;
		geraklurus();

		//if(j==10){comger==0;}
		}

	
}
}
#endif
