#include <iostream>
#include "dynamixel/dynamixel.h"
#include <unistd.h>
#include "movement/core_movement.h"

int main(){
	dxl_initialize(0,1);
	putarkanan();
	usleep(100);
}
