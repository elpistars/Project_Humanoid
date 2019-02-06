#ifndef ABD_C
#define ABD_C

#include "/home/pi/systemTest/library/allLib.h"

using namespace std;
int main ()
{

	while(1){
	string status;
	enableIMU();
	status=bacaACC(Pacc_raw);
	cout<<status<<endl;
	bacaMAG(Pmag_raw, 1);
	
	sleep(1);
	



 }  
 return 0;
 }
 #endif