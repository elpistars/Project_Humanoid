#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h> 
#include <string.h>

#include "movement/core_movement.h"
#include "kamera.cpp"

using namespace std;

IK r_ik,l_ik;

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
int jdt=0;
int jdt2;
void sensen(){

        string status;
        //enableIMU();
        //nilaisensor=bacaACC3(Pacc_raw);
        //cout<<status<<endl;
        //bacaMAG(Pmag_raw, 1);
        //nilaisensor = AccXangle;
//cout<<"a  "<<nilaisensor<<endl;
//sleep(1);

}

		
void jalan(){
   
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
   Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),100,512,512,480,520,270,750,250,750,x_head,y_head);

   f2=0;
   Invers.setParam(90,0.1,100);
   while(f2<=Invers.param.sudut){
        f2+=Invers.param.inc_sudut;
        Kanan.setParam(temp_joint[0].x+0*sin(f2*3.14/180),
        temp_joint[0].y-1.5*sin(f2*3.14/180),
        temp_joint[0].z+0*sin(f2*3.14/180),
        temp_joint[0].sudutX-4*sin(f2*3.14/180),
        temp_joint[0].sudutY+2*sin(f2*3.14/180),
        temp_joint[0].sudutZ+0*sin(f2*3.14/180)
        );
        
        Kiri.setParam(temp_joint[1].x+0*sin(f2*3.14/180),
        temp_joint[1].y-3*sin(f2*3.14/180),
        temp_joint[1].z+0*sin(f2*3.14/180),
        temp_joint[1].sudutX-8*sin(f2*3.14/180),
        temp_joint[1].sudutY+2*sin(f2*3.14/180),
        temp_joint[1].sudutZ+0*sin(f2*3.14/180)
        );

        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),100,512,512,480,520,270,750,250,750,x_head,y_head);

   }
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
   usleep(50000);
                         //  ANGKAT KAKI KIRI DAN MENDARAT //
   f2=0;
   Invers.setParam(180,0.03,150);
   while(f2<=Invers.param.sudut){
        f2+=Invers.param.inc_sudut; //0.05
	
        Kanan.setParam(temp_joint[0].x+0*sin(f2*3.14/180),
        temp_joint[0].y-2*sin(f2*3.14/180),
        temp_joint[0].z-0.5*sin(f2*3.14/180),
        temp_joint[0].sudutX+0*sin(f2*3.14/180),
        temp_joint[0].sudutY+0*sin(f2*3.14/180),
        temp_joint[0].sudutZ+0*sin(f2*3.14/180)
        );
        
        Kiri.setParam(temp_joint[1].x+2*sin(f2*3.14/180),
        temp_joint[1].y+0*sin(f2*3.14/180),
        temp_joint[1].z+0.5*sin(f2*3.14/180),
        temp_joint[1].sudutX-4*sin(f2*3.14/180),
        temp_joint[1].sudutY+0*sin(f2*3.14/180),
        temp_joint[1].sudutZ+0*sin(f2*3.14/180)
        );

        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),100,512,512,480,520,270,750,250,750,x_head,y_head);        
   }	
   f2=0;
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
   Invers.setParam(90,0.1,50);
   while(f2<=Invers.param.sudut){
	f2+=Invers.param.inc_sudut; //0.08
        
        Kanan.setParam(temp_joint[0].x+0*sin(f2*3.14/180),
        temp_joint[0].y-temp_joint[0].y*sin(f2*3.14/180),
        temp_joint[0].z+0*sin(f2*3.14/180),
        temp_joint[0].sudutX-temp_joint[0].sudutX*sin(f2*3.14/180),
        temp_joint[0].sudutY-3*sin(f2*3.14/180),
        temp_joint[0].sudutZ+0*sin(f2*3.14/180)
        );
        
        Kiri.setParam(temp_joint[1].x+0*sin(f2*3.14/180),
        temp_joint[1].y-temp_joint[1].y*sin(f2*3.14/180),
        temp_joint[1].z+0*sin(f2*3.14/180),
        temp_joint[1].sudutX*sin(f2*3.14/180),
        temp_joint[1].sudutY-3*sin(f2*3.14/180),
        temp_joint[1].sudutZ+0*sin(f2*3.14/180)
        );

        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),100,512,512,480,520,270,750,250,750,x_head,y_head);        

   }
   usleep(10000);
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
   		        // MIRING KIRI//

   f2=0;
   Invers.setParam(90,0.1,50);
   while(f2<=Invers.param.sudut){
        f2+=Invers.param.inc_sudut;
        Kanan.setParam(temp_joint[0].x+0*sin(f2*3.14/180),
        temp_joint[0].y+6*sin(f2*3.14/180),
        temp_joint[0].z+0*sin(f2*3.14/180),
        temp_joint[0].sudutX+0*sin(f2*3.14/180),
        temp_joint[0].sudutY+2*sin(f2*3.14/180),
        temp_joint[0].sudutZ+0*sin(f2*3.14/180)
        );
        
        Kiri.setParam(temp_joint[1].x+0*sin(f2*3.14/180),
        temp_joint[1].y+2*sin(f2*3.14/180),
        temp_joint[1].z+0*sin(f2*3.14/180),
        temp_joint[1].sudutX+0*sin(f2*3.14/180),
        temp_joint[1].sudutY+2*sin(f2*3.14/180),
        temp_joint[1].sudutZ+0*sin(f2*3.14/180)
        );
        
        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),100,512,512,480,520,270,750,250,750,x_head,y_head);
   }
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
   usleep(100000); //60000
				//ANGKAT KAKI KANAN DAN MENDARAT/
   f2=0;
   Invers.setParam(180,0.05,50);
   while(f2<=Invers.param.sudut){
        f2+=Invers.param.inc_sudut;
        Kanan.setParam(temp_joint[0].x+2*sin(f2*3.14/180),
        temp_joint[0].y+0*sin(f2*3.14/180),
        temp_joint[0].z+1*sin(f2*3.14/180),
        temp_joint[0].sudutX+0*sin(f2*3.14/180),
        temp_joint[0].sudutY+0*sin(f2*3.14/180),
        temp_joint[0].sudutZ+0*sin(f2*3.14/180)
        );
        
        Kiri.setParam(temp_joint[1].x+0*sin(f2*3.14/180),
        temp_joint[1].y+0*sin(f2*3.14/180),
        temp_joint[1].z-0.5*sin(f2*3.14/180),
        temp_joint[1].sudutX+0*sin(f2*3.14/180),
        temp_joint[1].sudutY+0*sin(f2*3.14/180),
        temp_joint[1].sudutZ+0*sin(f2*3.14/180)
        );
        
        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),100,512,512,480,520,270,750,250,750,x_head,y_head);
   }        
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
   usleep(60000);
		        // BALIKIN MIRING//
   f2=0;
   Invers.setParam(90,0.08,100);
   while(f2<=Invers.param.sudut){
        f2+=Invers.param.inc_sudut;
        Kanan.setParam(temp_joint[0].x+0*sin(f2*3.14/180),
        temp_joint[0].y-temp_joint[0].y*sin(f2*3.14/180),
        temp_joint[0].z+0*sin(f2*3.14/180),
        temp_joint[0].sudutX-temp_joint[0].sudutX*sin(f2*3.14/180),
        temp_joint[0].sudutY+3*sin(f2*3.14/180),
        temp_joint[0].sudutZ+0*sin(f2*3.14/180)
        );
        
        Kiri.setParam(temp_joint[1].x+0*sin(f2*3.14/180),
        temp_joint[1].y-temp_joint[1].y*sin(f2*3.14/180),
        temp_joint[1].z+0*sin(f2*3.14/180),
        temp_joint[1].sudutX*sin(f2*3.14/180),
        temp_joint[1].sudutY+3*sin(f2*3.14/180),
        temp_joint[1].sudutZ+0*sin(f2*3.14/180)
        );
        
        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),100,512,512,480,520,270,750,250,750,x_head,y_head);
   }
        f2=0;
    usleep(2000000);//usleep(300000); //usleep(300000)
/*							        // MIRING KANAN //

while(f2<=90)

        {f2=f2+0.08;
        sudutNgurang=f2;
        tempx=x;//+sin(sudutNgurang*3.14/180)*2;
        tempy=y-1*sin(sudutNgurang*3.14/180);//2.4
        tempz=z;//0.5;//cos(sudutNgurang*3.14/180);

        temp_sudutX=sudutX+2*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        temp_sudutY=sudutY+3*sin(sudutNgurang*3.14/180);
        temp_sudutZ=sudutZ;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));

        tempx2=x2;//-sin(sudutNgurang*3.14/180)*2.4;
        tempy2=y2-1*sin(sudutNgurang*3.14/180);//2.4
        tempz2=z2;//0.2;//cos(sudutNgurang*3.14/180);

        temp_sudutX2=sudutX2+2*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        temp_sudutY2=sudutY2+3*sin(sudutNgurang*3.14/180);
        temp_sudutZ2=sudutZ2;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=50;//cos(sudutNgurang*3.14/360);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	 inverseAndre(temp_sudutX,temp_sudutY,temp_sudutZ,tempx,tempy,tempz,temp_sudutX2,temp_sudutY2,temp_sudutZ2,tempx2,tempy2,tempz2,speed,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);                        }
        usleep(50000);

				//angkat kaki kiri dan mendarat
	f2=0;
	while(f2<=180){
        f2=f2+0.08;
        sudutNgurang=f2;
        x=tempx+0*sin(sudutNgurang*3.14/180);
        y=tempy-1*sin(sudutNgurang*3.14/180);//1+1*sin(sudutNgurang*3.14/180);//- pidy;//2.4
        z=tempz-1*sin(sudutNgurang*3.14/360);//sebelum -1

        sudutX=temp_sudutX;//-2*sin(sudutNgurang*3.14/180);//- pidx;//(pow(sin(sudutNgurang*3.14/180),2));
        sudutY=temp_sudutY;//sin(2*sudutNgurang*3.14/360);//+sin(sudutNgurang*3.14/180);
        sudutZ=temp_sudutZ;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=tempx2+2.5*sin(sudutNgurang*3.14/180);
        y2=tempy2-1*sin(sudutNgurang*3.14/180);//2.4
        z2=tempz2+1*sin(sudutNgurang*3.14/360); 			//2
        //z2=-1*cos(sudutNgurang*3.14/180);

        sudutX2=temp_sudutX2-14*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        sudutY2=temp_sudutY2;//+2*sin(2*sudutNgurang*3.14/360);
        sudutZ2=temp_sudutZ2;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//+50*cos(sudutNgurang*3.14/360);
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');

                                                        }

	sleep(5000);
 f2=0;
 while(f2<=90){
        f2=f2+0.08;
        sudutNgurang=f2;
        tempx=x;//+sin(sudutNgurang*3.14/180)*1.4;
        tempy=y-y*sin(sudutNgurang*3.14/180);//2.4
        tempz=z;//cos(sudutNgurang*3.14/180);

        temp_sudutX=sudutX-sudutX*sin(sudutNgurang*3.14/180);
        temp_sudutY=sudutY;//-1*sin(sudutNgurang*3.14/180);
        temp_sudutZ=sudutZ;//3*(pow(sin(sudutNgurang*3.14/180),2));

        tempx2=x2;//-sin(sudutNgurang*3.14/180)*1.4;
        tempy2=y2-y2*sin(sudutNgurang*3.14/180);//2.4
        tempz2=z2;//-0.5;//-0.5;//cos(sudutNgurang*3.14/180);

        temp_sudutX2=sudutX2-sudutX2*sin(sudutNgurang*3.14/180);
        temp_sudutY2=sudutY2;//-1*sin(sudutNgurang*3.14/180);
        temp_sudutZ2=sudutZ2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=50;//cos(sudutNgurang*3.14/360);

//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        inverseAndre(temp_sudutX,temp_sudutY,temp_sudutZ,tempx,tempy,tempz,temp_sudutX2,temp_sudutY2,temp_sudutZ2,tempx2,tempy2,tempz2,speed,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);

        }
//	cout<<"y :"<<y<<" y2: "<<y2<<endl;
	cout<<tempz<<endl<<tempz2<<endl;
	//tempz=1;tempz2=-1;
                f2=0;

usleep(500000);

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