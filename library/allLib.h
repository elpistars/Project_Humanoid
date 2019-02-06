#ifndef ALLLIB_H
#define ALLLIB_H

//------------------------------------------------ImageProc----------------------
#include <opencv2/objdetect/objdetect.hpp> // Library Opencv
#include <opencv2/core/core.hpp> // Library Opencv
#include <opencv2/highgui/highgui.hpp> // Library Opencv
#include <opencv2/imgproc/imgproc.hpp>// Library Opencv
#include <opencv2/opencv.hpp>// Library Opencv
//#include <conio.h>*/

//------------------------------------------------Sensor dan Dynamixel----------------------
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <unistd.h>
#include <time.h>

//------------------------------------------------MultiThreading-----------------------
#include <pthread.h>

//------------------------------------------------Dynamixel-----------------------------
#include <termio.h>
#include <termios.h>
//#include "/home/pi/systemTest/library/dynamixel/dynamixel.h"
#include <dynamixel.h>

//#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"
		// yang di komenbuat inverse kinematik
//#include "/home/pi/systemTest/library/dynamixel/InversKinematik.cpp"

//#include "/home/pi/systemTest/library/movement/move.h"
//#include "/home/pi/systemTest/library/movement/moveArray.h"


//------------------------------------------------Sensor---------------------------------
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/time.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <stdint.h>

#include "/home/pi/systemTest/library/sensor/i2c-dev.h"
#include "/home/pi/systemTest/library/sensor/L3G.h"
#include "/home/pi/systemTest/library/sensor/LSM303.h"
#include "/home/pi/systemTest/library/sensor/sensor.c"

//-----------------------------------------------WIFI-----------------------------------
#include <sys/types.h>
#include <sys/socket.h> 
#include <netinet/in.h>
#include <netdb.h>
#include <sstream>
#include <inttypes.h>
#include "/home/pi/systemTest/library/wifi/wifi.cpp"


using namespace std;
using namespace cv;

int comger = 1;
int comkir = 1;
int comkan = 1;

#endif
