#include "/home/pi/systemTest/library/allLib.h"
#include "i2c-dev.h"
#include "sensor.c"

int main(){
	string status;
	enableIMU();
	while(1){
		bacaACC(Pacc_raw);
		//cout << status << endl;
		bacaMAG(Pmag_raw, 1);
		//cout << status << endl;
		usleep(1000000);
	}
	return 0;
}
