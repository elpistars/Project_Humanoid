
#ifndef PUTARKIRI_CPP
#define PUTARKIRI_CPP

#include <iostream>
#include <unistd.h>
#include "movement/foot_ik.h"

using namespace std;


void spki(){
  double f2;
  double a=0.2;
        double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
        int speed, com7, com8;

	x=-12.4;
        y=0;// miring kaki
        z=0.5;// langkah kaki 

        sudutX=0;//miring badan
        sudutY=20;//kebungkukan badan 
        sudutZ=0;//putaran badan

        x2=-12; //tinggi kaki kiri
        y2=0;
        z2=-0.3;

        sudutX2=0;//
        sudutY2=20; //
        sudutZ2=0;

//        dxl_write_word(3,30,300);
  //      dxl_write_word(4,30,724);
        // inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
         inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
        cout<<"Masukkan Speed: ";
        //cin>>speed;

        sleep(2);//5

}
void putarkiri(){
  double f2;
  double a=0.2;

        double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
        int speed, com7, com8;
       						 // MIRING //
	x=-12.4;
        y=0;// miring kaki
        z=0.3;// langkah kaki 

        sudutX=0;//miring badan
        sudutY=22;//kebungkukan badan 
        sudutZ=0;//putaran badan

        x2=-12; //tinggi kaki kiri
        y2=0;
        z2=0;

        sudutX2=0;//
        sudutY2=22; //
        sudutZ2=0;


        f2=0;
        while(f2<=90.0)
   {
        f2=f2+0.05;
        sudutNgurang=f2;
        x=-12.4;//-2*sin(sudutNgurang*3.14/180)*2;
        y=-2*sin(sudutNgurang*3.14/180);//2.4
        z=0.3;
        sudutX=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=22;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-12;//+sin(sudutNg
		temp_joint[1].sudutX - 3*sin(f2*3.14/180),
		temp_joint[1].sudutY
		temp_joint[1].sudutZ + 472*sin(f2*3.14/180)urang*3.14/180)*2;
	y2=-2*sin(sudutNgurang*3.14/180);//2.4
        z2=0;
        sudutX2=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=22;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);

   }
        f2=0;
	//sleep(5000);
        usleep(1000);
// ------------------  ANKAT KAKI DAN MENDARAT //
        while(f2<=180)
        {
                f2=f2+0.1; 	//0.7
                sudutNgurang=f2;
                x=-12.4;//+2*sin(sudutNgurang*3.14/180);
		y=-2-0*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
                z=0.3;//0+0.8-0.8*cos(sudutNgurang*3.14/180);
                sudutX=2-4*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
                sudutY=22;//+6*sin(sudutNgurang*3.14/360);
                sudutZ=512;//+40*sin(sudutNgurang*3.14/360);;//+20*sin(sudutNgurang*3.14/360);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
                x2=-12+1*sin(sudutNgurang*3.14/180);
                y2=-2;//-1*sin(sudutNgurang*3.14/180);// + pidy;//sin(sudutNgurang*3.14/180);//2.4
                z2=0;//-0.8+0.8*cos(sudutNgurang*3.14/180);
                sudutX2=2-4*sin(sudutNgurang*3.14/180);// - pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
                sudutY2=22;//-4*sin(sudutNgurang*3.14/360);
                sudutZ2=512-50*sin(sudutNgurang*3.14/360);//-3*(pow(sin(sudutNgurang*3.14/180),2));		//512-40
                speed=250;//+50*cos(sudutNgurang*3.14/360);
                inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,speed,sudutZ,sudutZ2,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);

        }

        f2=0;
	usleep(5000);
        //usleep(1000);
// --------------------------- BALIKIN MIRING //
        while(f2<=90)
        {
                f2=f2+a;
                sudutNgurang=f2;
                x=-12.4;//-sin(sudutNgurang*3.14/180)*2;
                y=-2+2*sin(sudutNgurang*3.14/180);//2.4
                z=0.3;//1.6;
                sudutX=2-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
                sudutY=22;//-6*sin(sudutNgurang*3.14/180);
                sudutZ=512;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
                x2=-12;//+sin(sudutNgurang*3.14/180)*2;
                y2=-2+2*sin(sudutNgurang*3.14/180);//2.4
                z2=0;//-1.6;//0.2;//.2;
                sudutX2=2-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
                sudutY2=22;//+4*sin(sudutNgurang*3.14/180);
                sudutZ2=462;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2		//472
                speed=250;//cos(sudutNgurang*3.14/360);
                inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,speed,sudutZ,sudutZ2,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);

 }
        f2=0;
	usleep(700000); //800000
        //sleep(1000);

// ----------------------------- MIRING //
                while(f2<=90)
                {
                        f2=f2+0.01;
                        sudutNgurang=f2;
                        x=-12.4;//+sin(sudutNgurang*3.14/180)*2;
                        y=2*sin(sudutNgurang*3.14/180);		//1.5
                        z=0.3;//0.5;//cos(sudutNgurang*3.14/180);
                        sudutX=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
                        sudutY=22;//+1*sin(sudutNgurang*3.14/180);
                        sudutZ=512;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
                        x2=-12;//-sin(sudutNgurang*3.14/180)*2.4;
                        y2=2*sin(sudutNgurang*3.14/180);		//1.5
                        z2=0;//0.2;//cos(sudutNgurang*3.14/180);
                        sudutX2=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
                        sudutY2=22;//+1*sin(sudutNgurang*3.14/180);
                        sudutZ2=462;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));		//472
                        speed=100;//cos(sudutNgurang*3.14/360);
                        inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,speed,sudutZ,sudutZ2,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);

                }
		//sleep(5000);
                usleep(5000);
// ------------------------ ANGKAT KAKI DAN MENDARAT/
                f2=0;

                while(f2<=180){
                        f2=f2+0.1; //0.7
                        sudutNgurang=f2;
                        x=-12.4+10*sin(sudutNgurang*3.14/180);		//2
                        y=2+0*sin(sudutNgurang*3.14/180);//- pidy;			//ikutin miring
                        z=0.3;//-0.4-0.4*cos(sudutNgurang*3.14/180);
			sudutX=-2-6*sin(sudutNgurang*3.14/180);//- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/ 	//-2-4
                        sudutY=22;//-4*sin(sudutNgurang*3.14/360);//+sin(sudutNgurang*3.14/180)
                        sudutZ=512;//-40*sin(sudutNgurang*3.14/360);//3*(pow(sin(sudutNgurang*3.14/180),2)); //552-40
                        x2=-12+0*sin(sudutNgurang*3.14/180);
                        y2=2+2*sin(sudutNgurang*3.14/180);//2.4				//ikutin miring
                        z2=0;//0.4-0.4*cos(sudutNgurang*3.14/180);
                        sudutX2=2-6*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	//2-4
                        sudutY2=22;//+2*sin(sudutNgurang*3.14/360);
                        sudutZ2=462+50*sin(sudutNgurang*3.14/360);//-3*(pow(sin(sudutNgurang*3.14/180),2)); 	//472+40
                        speed=250;//+50*sin(sudutNgurang*3.14/180);
                        inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,speed,sudutZ,sudutZ2,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);

                }
		usleep(5000);
                f2=0;
                // BALIKIN MIRING//

                while(f2<=90){
                        f2=f2+0.1;
                        sudutNgurang=f2;
                        x=-12.4;//+sin(sudutNgurang*3.14/180)*1.4;
                        y=-2+1.5*sin(sudutNgurang*3.14/180);				//ikutin miring
                        z=0.3;//cos(sudutNgurang*3.14/180);//0
                        sudutX=-2+4*sin(sudutNgurang*3.14/180);
                        sudutY=22;//-4+4*sin(sudutNgurang*3.14/180);
                        sudutZ=512;//3*(pow(sin(sudutNgurang*3.14/180),2));
                        x2=-12;//-sin(sudutNgurang*3.14/180)*1.4;
                        y2=-2+1.5*sin(sudutNgurang*3.14/180);				//ikutin miring
                        z2=0;//0.8;//cos(sudutNgurang*3.14/180);
                        sudutX2=2-4*sin(sudutNgurang*3.14/180);
                        sudutY2=22;//+2*sin(sudutNgurang*3.14/180);
                        sudutZ2=512;//-3*(pow(sin(sudutNgurang*3.14/180),2)); 		//512
                        speed=250;//cos(sudutNgurang*3.14/360);
                        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
                        inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,speed,sudutZ,sudutZ2,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
}


                f2=0;
                usleep(500);

        //      int id[] = {20};
        //      int data1[][18] = {512};
        //      int delay1[] = {1000};
        //      int speed1[] = {100};
//              kirimPacketGerak(1, 1, 1, id, data1, 1, delay1, speed1);

}

#endif


