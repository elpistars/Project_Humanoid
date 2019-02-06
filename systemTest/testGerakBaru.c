#ifndef TESTGERAKBARU_C
#define TESTGERAKBARU_C

//#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/library/dynamixel/InversKinematik.cpp"
//#include "math.h"

using namespace std;

int main(){
	//ELP_initialize(0,1);
	//ELP_StatReturnSet(2);
	//setAllSpeed(120);
	//ELP_SyncHead(512, 512, 0);
	//usleep(2000000);

	int result;
                result = dxl_initialize(0,1);
                if(result == 1){
                        printf("Sukses buka koneksi :)");
                }else{
                        printf("Gagal buka koneksi :(");
                }

	int id[] = {1, 2, 3, 4, 5, 6,
		    7, 8, 9, 10, 11, 12,
		    13, 14, 15, 16, 17, 18};
	//int data[] = {600, 512, 300, 300, 325, 200};
	int data[][18] = {
			{600, 512, 300, 300, 425, 500},
			{600, 512, 300, 300, 125, 500},
			{600, 512, 300, 300, 425, 500},
			{600, 512, 300, 300, 150, 500},
			{600, 512, 300, 300, 425, 500}
			};
	int delay[] = {500000, 500000, 500000, 500000, 500000, 500000,
		       500000, 500000, 500000, 500000, 500000, 500000,
		       500000, 500000, 500000, 500000, 500000, 500000};
	int speed[] = {300, 300, 300, 300, 300, 300,
		       300, 300, 300, 300, 300, 300,
		       300, 300, 300, 300, 300, 300};

	while(1){
		//int result;
		//result = dxl_initialize(0,1);
		//if(result == 1){
		//	printf("Sukses buka koneksi :)");
		//}else{
		//	printf("Gagal buka koneksi :(");
		//}
		//kirimPacketSpeed(6, id, 500);
		//kirimPacketGerak(6, 1, 1, id, data, 1, 100);
		
		double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;

	        x=-10;//-sin(sudutNgurang*3.14/180)*2;
	        y=0;
	        z=0;
	        sudutX=0; //0
	        sudutY=8; //8
	        sudutZ=0; //0

	        x2=-10;
	        y2=0;
	        z2=0;
	        sudutX2=0; //0
	        sudutY2=8; //8
	        sudutZ2=0; //0

		kirimPacketInverse(sudutX, sudutY, sudutZ, x, y, z,sudutX2, sudutY2, sudutZ2, x2, y2, z2, delay, speed);
		
		//kirimPacketGerak(6, 5, 1, id, data, 1, delay, speed);
		usleep(1000000);
	}
	return 0;
}

#endif
