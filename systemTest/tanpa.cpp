#ifndef TANPA_CPP
#define TANPA_CPP

#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/putarkanan.cpp"
#include "/home/pi/systemTest/putarkiri.cpp"
#include "/home/pi/systemTest/garis.cpp"

int comger,detak;
int delayawal=5000000;
int delayjalan=500000;

int main(){
	int result;
	result = dxl_initialize(0,1);
        if(result == 1){	
			printf("Sukses");
        }else{
            printf("Gagal");
	}
	
	dxl_write_word(20,30,512); // kamera ke posisi tengah
	dxl_write_word(19,30,215);

   	if(!cap.isOpened()) // kondisi dimana jika kamera tidak terdeteksi maka program akan tertutup
     	return -1;
 
	comimage=0;
	comnentu=0;

	dxl_write_word(20,32,1023); // set speed servo 
	dxl_write_word(19,32,1023);
	delayberdiri=delayawal;
	berdiri();	
	delayberdiri=delayjalan;
	
while(1){	
/*cap.read(imgOriginal);
	for( int i=0; i<=2;i++){
	balls();
	}
	if(modegaris==1){
		for( int i=0; i<=2;i++){
		penggaris();
		}
	}
	switch(comimage){
	case 0:
		berdiri();
		break;
	case 1:
		langkah();
		berdiri();
		break;
	case 2:
		putarkanan();
		berdiri();
		break;
	case 3:
		putarkiri();
		berdiri();
		break;
	case 4:
		for(int i=0;i<=5;i++){
		putarkanan();
		berdiri();}
		modeputar=0;
		break;
	case 5:
		for(int i=0;i<=5;i++){
		putarkiri();
		berdiri();}
		modeputar=0;
		break;
	}*/

	putarkanan();	
	}
}
#endif
