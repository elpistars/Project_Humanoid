#include <iostream>
#include "dynamixel/dynamixel.h"
#include <unistd.h>
#include "movement/core_movement.h"

using namespace std;
int main(){
	dxl_initialize(0,1);
	cout<<x_head<<endl<<y_head;
	for(int i=0; i<10; i++){
	  berdiri();
	  usleep(1000);
	jalan();
	usleep(1000);  
	}
}
