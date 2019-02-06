#ifndef TEST1_C
#define TEST1_C

//#include "/home/pi/systemTest/library/allLib.h"
#include "InversKinematik.cpp"

using namespace std;

int main(){

	int id[] = {1, 2};
	int data[] = {300, 600};
	int delay[] = {50000, 50000};
	int speed[] = {300, 300};

/*	for(i=0; i<2; i++){
		cout << id[i] << endl;
		cout << data[i] << endl;
		cout << delay[i] << endl;
		cout << speed[i] << endl;
	}*/

	while(1){
		kirimPacketGerak(2, 1, 1, id, data, 1, delay, speed);
		usleep(1000000);
	}

//	return 0;
}
#endif
