#include <iostream>
#include "dynamixel/dynamixel.h"
#include <unistd.h>
#include "movement/core_movement.h"

using namespace std;
int main(){
	dxl_initialize(0,1);
	cout<<x_head<<endl<<y_head;
	berdiri();
	usleep(100);
}
