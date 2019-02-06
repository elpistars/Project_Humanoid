#ifndef COBARDAN_C
#define COBARDAN_C

#include "/home/pi/systemTest/library/allLibtune.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

#include <iostream>
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h> 

double nilaisensor,epx,edx,epy,edy,erry,outpx,outpy;
double kpx = 0;
double kdx = 0;//.005;;
double kpy = 0;
double kdy = 0;
double outdx = 0;
double outdy = 0;
double edysblm = 0;
double edxsblm = 0;
double pidx = 0;
double pidy= 0;
double setpointx = 270.5;
double setpointy = 1.1;
double f2;
double a=0.2;
struct {
	int condition,x_head,y_head;
}data_image;
int jdt=0;
int jdt2;
void sensen(){

        string status;
       // enableIMU();
       // nilaisensor=bacaACC3(Pacc_raw);
        //cout<<status<<endl;
        //bacaMAG(Pmag_raw, 1);
        //nilaisensor = AccXangle;
//cout<<"a  "<<nilaisensor<<endl;
//sleep(1);

}


void berdiri(){

        double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
        int speed, com7, com8;


//       com7=4;
  //      com8=4;
              dxl_write_word(10,26,1);
             dxl_write_word(10,27,1);
        dxl_write_word(9,26,1);
        dxl_write_word(9,27,1);

        //kirimPacketTorque(12,id,tor);
        //for(int i=0;i<=11;i++){
          dxl_write_word(10,28,4);
          dxl_write_word(10,29,4);
	  dxl_write_word(9,28,4);
	  dxl_write_word(9,29,4);



/*
        cout<<"Masukkan sudut X: ";
        cin>>sudutX;
        cout<<"Masukkan sudut Y: ";
        cin>>sudutY;
        cout<<"Masukkan sudut Z: ";
        cin>>sudutZ;

        cout<<"Masukkan X: ";
        cin>>x;
        cout<<"Masukkan Y: ";
        cin>>y;
        cout<<"Masukkan Z: ";
        cin>>z;*/
        x=-13.6;// tinggi kaki kanan //-10.6
        y=-0.3;// miring kaki
        z=0;// langkah kaki //0

        sudutX=3;//miring badan
        sudutY=14;//kebungkukan badan//-6 
        sudutZ=0;//putaran badan

        x2=-13.4; //tinggi kaki kiri//10.4
        y2=0.2;//3;
        z2=0;

        sudutX2=2.5;//
        sudutY2=14; //-6
        sudutZ2=0;

        dxl_write_word(3,30,300);
        dxl_write_word(4,30,724);
        // inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
         inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);
        cout<<"Masukkan Speed: ";
        //cin>>speed;
        sleep(1);//5
         //kirimPacketInverse(x,y);
        //cin>>speed;
        //for(int i=1;i<=10;i++){
		}
		
void jalan(){
	 double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
        int speed, com7, com8;
			//-----------------------------------------------------------------------------
							        // MIRING KANAN//
        f2=0;
        while(f2<=90.0)
         //for(int i=0;i<=38;i++)
                {f2=f2+0.01;
        //cout<<"Masukkan X: ";
         sudutNgurang=f2;

        x=-13.6;//-2*sin(sudutNgurang*3.14/180)*2;
        y=-0.3+1*sin(sudutNgurang*3.14/180);//
        z=0; //2.5

        sudutX=3+2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=14;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-13.4;//+sin(sudutNgurang*3.14/180)*2;
        y2=0.2+1*sin(sudutNgurang*3.14/180);//-1.5
        z2=0;

        sudutX2=2.5+2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=14;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));

        speed=100;//cos(sudutNgurang*3.14/360);

        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
         inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);
//      usleep(1000);
//         cout<<"Masukkan sudut X: "<<f2<<endl;
        }
		
        f2=0;
	//sleep(5); 
	sleep(5000);
       							 //  ANGKAT KAKI KIRI DAN MENDARAT //
 	//cout<<"Masukkan sudut X: ";
         //for(int i=0;i<=76;i++)
    while(f2<=180){
		f2=f2+(0.01);

	sudutNgurang=f2;
        x=-13.6+0*sin(sudutNgurang*3.14/180); //+2
        y=-0.3+1+1*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//1.5
        z=-0.5+0.5*cos(sudutNgurang*3.14/180); //0+0.5-0.5*cos(sudutNgurang*3.14/180);1

        sudutX=3+2;//+8*sin(sudutNgurang*3.14/180);//*(pow(sin(sudutNgurang*3.14/180),2));*//0
        sudutY=14;//-2*sin(sudutNgurang*3.14/360); /-3
        sudutZ=0;//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-13.4+2*sin(sudutNgurang*3.14/180); //-11.4-0
        y2=0.2+1+1*sin(sudutNgurang*3.14/180);// + pidy;//sin(sudutNgurang*3.14/180);//2.5+2
        z2=0.5-0.5*cos(sudutNgurang*3.14/180);//-0.5 //+0.5

        sudutX2=2.5+2+7*sin(sudutNgurang*3.14/180);// - pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/-2 //9+16
        sudutY2=14;//-2*cos(sudutNgurang*3.14/360);//-3
    	sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2))//0
        speed=100;//+50*cos(sudutNgurang*3.14/360);

        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        //baca sensor

	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);
        //sensen();
//              if(i>46){
                        //PID x
                        // P x
                              // P y
}
     usleep(500);

        f2=0;
         //cout<<"Masukkan sudut X: ";
								        // BALIKIN MIRING //
 	//for(int i=0;i<=38;i++)
        while(f2<=90)
        {f2=f2+0.006;
         sudutNgurang=f2;
        x=-13.6;//-sin(sudutNgurang*3.14/180)*2;
        y=0.3+1-1*sin(sudutNgurang*3.14/180);//2.4
        z=-1;

        sudutX=3+2-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=14;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-13.4;//+sin(sudutNgurang*3.14/180)*2;
        y2=0.2+1-1*sin(sudutNgurang*3.14/180);//2.4
        z2=1;//0.2;//.2;

        sudutX2=2.5+2-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=14;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2
        speed=50;//cos(sudutNgurang*3.14/360);

//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
         inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);
        //usleep(1000);
//       cout<<"Masukkan sudut X: ";
                }
        f2=0;

         //cout<<"Masukkan sudut X: ";
         usleep(10000);


							        // MIRING KIRI//
        for(int j=0;j<=76;j++){
        while(f2<=90)

        {f2=f2+a;
        sudutNgurang=f2;
        x=-13.6;//+sin(sudutNgurang*3.14/180)*2;
        y=-0.3-1*sin(sudutNgurang*3.14/180);//2.4
        z=1;//0.5;//cos(sudutNgurang*3.14/180);

        sudutX=3-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
 	sudutY=14;//+1*sin(sudutNgurang*3.14/180);
        sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-13.4;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=0.2-1*sin(sudutNgurang*3.14/180);//2.4
        z2=1;//0.2;//cos(sudutNgurang*3.14/180);

        sudutX2=2.5-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=14;//+1*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);
                        }
        usleep(50000);
						        //ANGKAT KAKI KANAN DAN MENDARAT/
        f2=0;
        //======================
        outdx = 0;
        outdy = 0;
        edysblm = 0;
        edxsblm = 0;
        pidx = 0;
        pidy= 0;
        //==================
        while(f2<=180){
        f2=f2+0.009;
        sudutNgurang=f2;
        x=-13.6+2*sin(sudutNgurang*3.14/180);
        y=-0.3-1-1*sin(sudutNgurang*3.14/180);//1+1*sin(sudutNgurang*3.14/180);//- pidy;//2.4
        z=-1*cos(sudutNgurang*3.14/180);

        sudutX=3-2+17*sin(sudutNgurang*3.14/180);//- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=14;//+0*sin(sudutNgurang*3.14/180);//+sin(sudutNgurang*3.14/180);
        // speed=200*cos(sudutNgurang*3.14*0.5/180);
        sudutZ=0;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-13.4+0*sin(sudutNgurang*3.14/180);
        y2=0.2-1-0*sin(sudutNgurang*3.14/180);//2.4
        z2=1*cos(sudutNgurang*3.14/180);
 	//z2=-1*cos(sudutNgurang*3.14/180);

        sudutX2=2.5-2+2*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=14;//+2*sin(2*sudutNgurang*3.14/360);
        sudutZ2=0;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//+50*cos(sudutNgurang*3.14/360);
 	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
//		usleep(500);    
    			//cout<<x<<endl;
		}
        usleep(5000);
        f2=0;
						        // BALIKIN MIRING//

        while(f2<=90){
        f2=f2+a;
        sudutNgurang=f2;
        x=-13.6;//+sin(sudutNgurang*3.14/180)*1.4;
        y=-0.3-1+1*sin(sudutNgurang*3.14/180);//2.4
        z=1;//cos(sudutNgurang*3.14/180);

        sudutX=3+2-2*sin(sudutNgurang*3.14/180);
        sudutY=14;//-1*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-13.4;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=0.2-1+1*sin(sudutNgurang*3.14/180);//2.4
        z2=-1;//-0.5;//-0.5;//cos(sudutNgurang*3.14/180);

        sudutX2=2.5+2-2*sin(sudutNgurang*3.14/180);
        sudutY2=14;//-1*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);

//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);

                }
                f2=0;
                //usleep(500000);
                sleep(400000);
							        // MIRING KANAN //
while(f2<=90)

        {f2=f2+0.01;
        sudutNgurang=f2;
        x=-13.6;//+sin(sudutNgurang*3.14/180)*2;
        y=-0.3+1*sin(sudutNgurang*3.14/180);//2.4
        z=1;//0.5;//cos(sudutNgurang*3.14/180);

        sudutX=3+2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=14;//+1*sin(sudutNgurang*3.14/180);
        sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-13.4;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=0.2+1*sin(sudutNgurang*3.14/180);//2.4
        z2=-1;//0.2;//cos(sudutNgurang*3.14/180);

        sudutX2=2.5+2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=14;//+1*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);                        }
        usleep(50000);

	f2=0;
	while(f2<=180){
        f2=f2+0.009;
        sudutNgurang=f2;
        x=-13.6+0*sin(sudutNgurang*3.14/180);
        y=-0.3+1+1*sin(sudutNgurang*3.14/180);//1+1*sin(sudutNgurang*3.14/180);//- pidy;//2.4
        z=1-2*sin(sudutNgurang*3.14/360);

        sudutX=3+2+3*sin(sudutNgurang*3.14/180);//- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=14;//sin(2*sudutNgurang*3.14/360);//+sin(sudutNgurang*3.14/180);
        // speed=200*cos(sudutNgurang*3.14*0.5/180);
        sudutZ=0;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-13.4+3*sin(sudutNgurang*3.14/180);
        y2=0.2+1+3*sin(sudutNgurang*3.14/180);//2.4
        z2=-1+2*sin(sudutNgurang*3.14/360);
        //z2=-1*cos(sudutNgurang*3.14/180);

        sudutX2=2.5+2+10*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=14;//+2*sin(2*sudutNgurang*3.14/360);
        sudutZ2=0;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//+50*cos(sudutNgurang*3.14/360);
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');

                                                        }

usleep(50000);

 while(f2<=90){
        f2=f2+a;
        sudutNgurang=f2;
        x=-13.6;//+sin(sudutNgurang*3.14/180)*1.4;
        y=-0.3+1-1*sin(sudutNgurang*3.14/180);//2.4
        z=-1;//cos(sudutNgurang*3.14/180);

        sudutX=3+2-2*sin(sudutNgurang*3.14/180);
        sudutY=14;//-1*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-13.4;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=0.2+1-1*sin(sudutNgurang*3.14/180);//2.4
        z2=1;//-0.5;//-0.5;//cos(sudutNgurang*3.14/180);

        sudutX2=2.5+2-2*sin(sudutNgurang*3.14/180);
        sudutY2=14;//-1*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);

//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);

                }
                f2=0;

usleep(500000);
}
/*

         while(f2<=90){
        f2=f2+a;
        sudutNgurang=f2;
        x=-10.4;//+sin(sudutNgurang*3.14/180)*2;
        y=2*sin(sudutNgurang*3.14/180);//2.4;
        z=0.5;//cos(sudutNgurang*3.14/180);

        sudutX=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
  /*      sudutY=-5;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-10.4;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=2*sin(sudutNgurang*3.14/180);//2.4
        z2=-0.5;//-0.2;//cos(sudutNgurang*3.14/180);

        sudutX2=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
    /*    sudutY2=-5;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
                        }

   	usleep(5000);
        						//ANGKAT KAKI DAN MENDARAT//
        f2=0;
        //=======================
        outdx = 0;
        outdy = 0;
        edysblm = 0;
        edxsblm = 0;
        pidx = 0;
        pidy= 0;
        //=======================

        while(f2<=180){
        f2=f2+a;
        sudutNgurang=f2;

        x=-13.4;//+2*sin(sudutNgurang*3.14/180);
	y=2;//-0*sin(sudutNgurang*3.14/180);//2.4
        z=1-1*sin(2*sudutNgurang*3.14/180);

        sudutX=-2;//+12*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
    /*    sudutY=-5+0*sin(sudutNgurang*3.14/180);
        sudutZ=0;//cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-13.4+1*sin(sudutNgurang*3.14/180)*1.5;
        y2=2;//-2*sin(sudutNgurang*3.14/180);//+pidy;//2.4
        z2=-1+1*sin(2*sudutNgurang*3.14/180);

        sudutX2=-2+16*sin(sudutNgurang*3.14/180);//- pidx;//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
   /*     sudutY2=-5+0*sin(sudutNgurang*3.14/180);//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//1+100*cos(sudutNgurang*3.14/360);
	//        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
 	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500);
         //sensen();
	//      while(i>=26){
        //PID x
                // P x
       /* epx = setpointx - nilaisensor;
        outpx = kpx*epx;
        //D x
        edx = epx - edxsblm;
        outdx = kdx*edx;
        edxsblm = epx;

        pidx = outpx + outdx;//PID y
        // P y
        erry = cos(epx) + sin(epx);
        epy = setpointy - erry;
        outpy = kpy*epy;
        // D y
        edy = epy - edysblm;
        outdy = kdy*edy;
        edysblm = epy;

        pidy = outpy + outdy;
        //cout<<"a  "<<nilaisensor<<endl;
*/
	//usleep(5000);
 /*}
	f2=0;
   	usleep(5000);
        // BALIKIN MIRING//
        while(f2<=90){
        f2=f2+a;
        sudutNgurang=f2;

        x=-10.4;//+n(sudutNgurang*3.14/180)*1.4;
        y=2-2*sin(sudutNgurang*3.14/180);//2.4
        z=0;//cos(sudutNgurang*3.14/180);

        sudutX=-2+2*sin(sudutNgurang*3.14/180);
        sudutY=-5;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//-2;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=-10.4;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=2-2*sin(sudutNgurang*3.14/180);//2.4
        z2=0;//0.2;//cos(sudutNgurang*3.14/180);

        sudutX2=-2+2*sin(sudutNgurang*3.14/180);
        sudutY2=-5;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=50;//*cos(sudutNgurang*3.14/360);
//        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,512,512,'2',500,500,250,780,500,500);

                }*/
//   cout<<nilaisensor<<endl;
//sleep(500000);
//        usleep(80000);
       
//    usleep(30000);
}

#endif
int main(){
	int fd;
	dxl_initialize(0,1);
//        dxl_write_word(19,32,800);
//        dxl_write_word(20,32,800);
        dxl_write_word(19,30,250);
        dxl_write_word(20,30,512);
	data_image.y_head=250;
	data_image.x_head=512;
	data_image.condition=1;	
	berdiri();
	sleep(1);
	char * myfifo="/tmp/myfifo";
//	mkfifo(myfifo,0666);
	while(1){
//		fd=open(myfifo,O_RDONLY);
//		read(fd,&data_image,sizeof(data_image));
//		close(fd);
//		std::cout<<"cmd_img "<<data_image.condition<<endl<<endl;
		data_image.condition=1;
		switch(data_image.condition){
		case 0:
			dxl_write_word(19,30,data_image.y_head);
			dxl_write_word(20,30,data_image.x_head);
			cout<<data_image.x_head<<" || "<< data_image.y_head<<endl;
			berdiri();
			break;
		case 1:
			jalan();
			break;
		}
	}
	close(fd);
	return 0;
}

