#ifndef COBAPUSING_C
#define COBAPUSING_C

#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"
#include <stdio.h>

using namespace std;

int main(){
	
	int id[]={19];
	while(1){
		kirimPaketGerak(1, id, 400);
	}
	return 0;
}

#endif
