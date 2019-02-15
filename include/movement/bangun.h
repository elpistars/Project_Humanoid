#ifndef BANGUN_H
#define BANGUN_H
#include "dynamixel/syncWrite.cpp"

#include "dynamixel/dynamixel.h"
#include "moveArray.h"
//#include "/home/pi/systemTest/library/dynamixel/InversKinematik.cpp"
//#include "/home/pi/systemTest/library/sensor/sensor.c"

///////////////////////////////////define end here//////////////////////////////////
int bKa1;
int bKi1;
int bKa2;
int bKi2;
int bKa3;
int bKi3;
int bKa4;
int bKi4;
int bKa5;
int bKi5;
int bKa6;
int bKi6;



/*
	void bangun_belakang1()
{
printf ("gw sedang bangun belakang bro");
//        siapJalan(1,0);
        //set speed
        dxl_write_word(BROADCAST_ID,32,speed_UP_B[0]);
        kirimPacketGerak(18,11,1,id,UP_B,1,0);
        usleep (delay_UP_B[0]);

        //set head servo speed
        dxl_write_word(19,32,0);
        dxl_write_word(20,32,0);
}*/
void bangun_belakang2()
{
      //  siapJalan(1,0);
        dxl_write_word(19,30,512);
        dxl_write_word(20,30,512);

        kirimPacketGerak(18,6,1,id,UP_B,1,delay_UP_B,speed_UP_B);

}
/*void bangun_belakang3()
{
        //siapJalan(1,0);
        kirimPacketGerak(18,26,4,id,UP_B,1,delay_UP_B,speed_UP_B);

        dxl_write_word(19,32,0);
        dxl_write_word(20,32,0);
}*/
/*void bangun_belakang_laststep()
{
                dxl_write_word(1,32,200);
                dxl_write_word(2,32,200);
                dxl_write_word(3,32,200);
                dxl_write_word(4,32,200);
                dxl_write_word(5,32,200);
                dxl_write_word(6,32,200);
                dxl_write_word(7,32,200);
                dxl_write_word(8,32,200);
                dxl_write_word(9,32,200);
                dxl_write_word(10,32,200);
                dxl_write_word(11,32,150);
                dxl_write_word(12,32,150);
                dxl_write_word(13,32,180);
                dxl_write_word(14,32,180);
                dxl_write_word(15,32,140);
                dxl_write_word(16,32,140);
                dxl_write_word(17,32,180);
                dxl_write_word(18,32,180);
        //dxl_write_word(BROADCAST_ID,32,200);
        kirimPacketGerak(18,1,1,id,UP_B_LAST_STEP,1,0);
        usleep (delay_UP_F_LAST_STEP[0]);
        dxl_write_word(3,24,0);
        dxl_write_word(4,24,0);
        dxl_write_word(5,24,0);
        dxl_write_word(6,24,0);
        usleep(2000000);

}*/

#endif
