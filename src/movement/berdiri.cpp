#ifndef BERDIRI_CPP
#define BERDIRI_CPP

#include <iostream>
#include <unistd.h>
#include "movement/foot_ik.h"

void berdiri(){
    x_head=548;
    y_head=310;
    /*dxl_write_word(10,26,1);
    dxl_write_word(10,27,1);
    dxl_write_word(9,26,1);
    dxl_write_word(9,27,1);

    //kirimPacketTorque(12,id,tor);
    //for(int i=0;i<=11;i++){
    dxl_write_word(10,28,4);
    dxl_write_word(10,29,4);
    dxl_write_word(9,28,4);
	dxl_write_word(9,29,4);*/
    Kanan.setParam(-12.8, 0, 0.5, 0, 20, 0); //x, y, z, sudutX, sudutY, sudutZ
    Kiri.setParam(-12.4, 0, 0, 0, 20, 0);//sudut x untuk miring, sudut y buat bungkuk
    
    Kanan.printParam((char*) "Kanan:");
    Kiri.printParam((char*) "Kiri:");

    Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512,480,520,270,750,250,750,x_head,y_head);
}

#endif
