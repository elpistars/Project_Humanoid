#include "/home/pi/systemTest/library/allLibtune.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"
#include <iostream>
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h> 

#include <iomanip>
#include <sstream>

#include <chrono>
#include <Kalman.h>
#include <mpu.h>
#include <time.h>
#define K_Y_OFFSET 0.1
#define K_Z_OFFSET 0.5

#define RAD_TO_DEG 180/M_PI
Kalman kalmanX;
Kalman kalmanY;

double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;

/*double nilaisensor,epx,edx,epy,edy,erry,outpx,outpy;
double kpx = 0;
double kdx = 0;//.005;;
double kpy = 0;
*/
//double kdy = 0;
double outdx = 0;
double outdy = 0;
double edysblm = 0;
double edxsblm = 0;
/*double pidx = 0;
double pidy= 0;
double setpointx = 270.5;
double setpointy = 1.1;*/
double px,dx,ix,kpx,kdx,kix,errorx,sigmerx,errorxsem,sigsemx,pidx;
double py,dy,iy,kpy,kdy,kiy,errory,sigmery,errorysem,sigsemy,pidy;

double f2;
double a=0.2;
struct {
	int condition,x_head,y_head;
}data_image;
int jdt=0;
int jdt2;
void get_mpu(double &kalAngleX, double &kalAngleY);

//variable timer
auto t_start=std::chrono::high_resolution_clock::now();
auto t_end=std::chrono::high_resolution_clock::now();
std::chrono::duration<double> t_span;

double roll,pitch,kalX,kalY;
struct {
        double x,y;
}data_mpu;
void get_mpu(double &kalAngleX,double &kalAngleY){
  double dt;
  gyro_read();
  accel_read();
  t_end = std::chrono::high_resolution_clock::now();
  t_span=std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
  dt=t_span.count();
  t_start = std::chrono::high_resolution_clock::now();
  roll= atan(accel.y / sqrt(accel.x * accel.x + accel.z * accel.z)) * RAD_TO_DEG;
  pitch= atan2(-accel.x, accel.z) * RAD_TO_DEG;
  //if((pitch < -90 && kalAngleY > 90) || (roll > 90 &&kalAngleX < -90)){
  //  kalmanY.setAngle(pitch);
  //  kalAngleY = pitch;
 // }
 // else{
    kalAngleY = kalmanY.getAngle(pitch,gyro.y,dt);
  //}
  //if(abs(kalAngleY) > 90)
    //gyro.x=-gyro.x;
  kalAngleX = kalmanX.getAngle(roll, gyro.x, dt);
  std::cout<<std::setprecision(3)<<kalAngleX<<" || "<<kalAngleY<<" || "<<dt<<std::endl<<std::endl;
}

//void sensen(){

//        string status;
        //enableIMU();
        //nilaisensor=bacaACC3(Pacc_raw);
        //cout<<status<<endl;
        //bacaMAG(Pmag_raw, 1);
        //nilaisensor = AccXangle;
//cout<<"a  "<<nilaisensor<<endl;
//sleep(1);

//}


void berdiri(){

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



	   px=0;
        dx=0;
        ix=0;
    kpx=0.3;
      kdx=0.12;
      kix=0;
        errorx=0;
        sigmerx=0;
        errorxsem=0;
        sigsemx=0;

         py=0;
        dy=0;
        iy=0;
        kpy=0.13;
       kdy=0.12;
        kiy=0;
        errory=0;
        sigmery=0;
        errorysem=0;
        sigsemy=0;


/*
        cout<<"Masukkan sudut X: ";
        cin>>sudutX;
        cout<<"Masukkan sudut Y: ";
        cin>>sudutY;
        cout<<"Masukkan sudut Z: ";
        cin>>sudutZ;

        cout<<"Masukkan X: ";
a        cin>>x;
        cout<<"Masukkan Y: ";
        cin>>y;
        cout<<"Masukkan Z: ";
        cin>>z;*/
        x=-12.4;// tinggi kaki kanan //-13.4
        y=0;// miring kaki
        z=0.3;// langkah kaki

        sudutX=0;//miring badan
        sudutY=18;//kebungkukan badan
        sudutZ=0;//putaran badan

        x2=-12.4; //tinggi kaki kiri
        y2=0;
        z2=0;

        sudutX2=0;
        sudutY2=18;
        sudutZ2=0;
        cout<<sudutX<<endl<<sudutY<<endl<<sudutZ<<endl<<x
        <<endl<<y<<endl<<z<<endl;
        // inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
         inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);//250,780,500,500
        cout<<"Masukkan Speed: ";
        //cin>>speed;
	get_mpu(kalX,kalY);
	cout<<kalX<<endl;
        sleep(1);//5
         //kirimPacketInverse(x,y);
        //cin>>speed;
        //for(int i=1;i<=10;i++){
		}
		
void jalan(){
	double	tempx,tempy,tempz,temp_sudutX,temp_sudutY,temp_sudutZ,tempx2,tempy2,tempz2,temp_sudutX2,temp_sudutY2,temp_sudutZ2;
        int speed, com7, com8;
	x=-12.4;// tinggi kaki kanan //-13.4
        y=0;// miring kaki
        z=0.3;// langkah kaki 

        sudutX=0;//miring badan
        sudutY=18;//kebungkukan badan 
        sudutZ=0;//putaran badan

        x2=-12.4; //tinggi kaki kiri
        y2=0;
        z2=0;

        sudutX2=-0;
        sudutY2=18; 
        sudutZ2=0;
//sleep(1000);
 accel_read();
  roll= atan(accel.y / sqrt(accel.x * accel.x + accel.z * accel.z)) * RAD_TO_DEG;
  pitch= atan2(-accel.x, accel.z) * RAD_TO_DEG;
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyro.x=roll;
  gyro.y=pitch;
  t_start = std::chrono::high_resolution_clock::now();
			//-----------------------------------------------------------------------------
							        // MIRING KANAN//
        f2=0;
	while(1)
         //for(int i=0;i<=38;i++)
                {f2=f2+1;
	get_mpu(kalX,kalY);

	sudutNgurang=f2;
	errorx=kalX-70;//sin(sudutNgurang*3.14/180);
        sigmerx=errorx+sigsemx;
        px=kpx*errorx;
        dx=kdx*(errorx-errorxsem);
        errorxsem=errorx;
        ix=kix*(sigmerx);
        sigmerx=sigsemx;
        pidx=px+dx+ix;

         errory=kalY-0;
        sigmery=errory+sigsemy;
        py=kpy*errory;
        dy=kdy*(errory-errorysem);
        errorysem=errory;
        iy=kiy*(sigmery);
        sigmery=sigsemy;
        pidy=py+dy+iy;
 
       //cout<<"Masukkan X: ";
        // sudutNgurang=f2;
        tempx=x;//-2*sin(sudutNgurang*3.14/180)*2;
        tempy=y+K_Y_OFFSET*pidx;//-1*sin(sudutNgurang*3.14/180);//
        tempz=z+K_Z_OFFSET*pidy; //2.5

        temp_sudutX=sudutX+pidx;//-9*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        temp_sudutY=sudutY-pidy;//+2*sin(sudutNgurang*3.14/180);//+sin(sudutNgurang*3.14/180);
        temp_sudutZ=sudutZ;//3*(pow(sin(sudutNgurang*3.14/180),2));

        tempx2=x2-0*sin(sudutNgurang*3.14/180)*2;
        tempy2=y2+K_Y_OFFSET*pidx;//-4*sin(sudutNgurang*3.14/180);//-1.5
        tempz2=z2+K_Z_OFFSET*pidy ;

        temp_sudutX2=sudutX2+pidx;//-2*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        temp_sudutY2=sudutY2-pidy;//+2*sin(sudutNgurang*3.14/180);//-sin(sudutNgurang*3.14/180);
        temp_sudutZ2=sudutZ2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=50;//cos(sudutNgurang*3.14/360);
        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
         inverseAndre(temp_sudutX,temp_sudutY,0,tempx,tempy,tempz,temp_sudutX2,temp_sudutY2,0,tempx2,tempy2,tempz2,speed+pidx+pidy,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
      usleep(1000);
         cout<<"Masukkan sudut X: "<<pidx<<endl;
        }
		
        f2=0;
	//sleep(5); 
	
	sleep(50000);
       							 //  ANGKAT KAKI KIRI DAN MENDARAT //
 	//cout<<"Masukkan sudut X: ";
         //for(int i=0;i<=76;i++)
   	 while(f2<=180)
	{f2=f2+0.5; //0.1

	sudutNgurang=f2;
        x=tempx+0*sin(sudutNgurang*3.14/180); //+2	//
        y=tempy-1;//sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//		-1
        z=tempz-0.75*sin(sudutNgurang*3.14/360); //0+0.5-0.5*cos(sudutNgurang*3.14/180);	//  =0.5*sin(360)

        sudutX=temp_sudutX-0*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));//sebelum -4
        sudutY=temp_sudutY;//-2*sin(sudutNgurang*3.14/360); /-3
        sudutZ=temp_sudutZ;//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=tempx2+0*sin(sudutNgurang*3.14/180); 
        y2=tempy2;//-1.5*sin(sudutNgurang*3.14/180);// + pidy;//sin(sudutNgurang*3.14/180);//2.5+2	// -1
        z2=tempz2+0.75*sin(sudutNgurang*3.14/360);//-0.5 //+0.5 //  2*sin(360)

        sudutX2=temp_sudutX2-20*sin(sudutNgurang*3.14/180);// - pidx;(pow(sin(sudutNgurang*3.14/180),2));		//-20 BENER
        sudutY2=temp_sudutY2;//-2*cos(sudutNgurang*3.14/360);//-3
    	sudutZ2=temp_sudutZ2;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2))//0
        speed=150;//+50*cos(sudutNgurang*3.14/360);

        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        //baca sensor

	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,552,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);

//sensen();
//              if(i>46){
                        //PID x
                        // P x
                              // P y
}	
	f2=0;
	usleep(60000);
/*       usleep(500);////usleep(5000);
	 while(f2<=180)
        {f2=f2+(0.1); //0.1

        sudutNgurang=f2;
        x=tempx+0*sin(sudutNgurang*3.14/180);
        y=tempy-1*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//1.5
        z=tempz-0.5*sin(sudutNgurang*3.14/360); //0+0.5-0.5*cos(sudutNgurang*3.14/180); //-1.6

        sudutX=temp_sudutX+0*sin(sudutNgurang*3.14/360);
        sudutY=temp_sudutY;//-2*sin(sudutNgurang*3.14/360); /-3
        sudutZ=temp_sudutZ;//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=tempx2+5*sin(sudutNgurang*3.14/180); 
        y2=tempy2-1*sin(sudutNgurang*3.14/180);// + pidy;//sin(sudutNgurang*3.14/180);//2.5+2  
        z2=tempz2+1.5*sin(sudutNgurang*3.14/360);//-0.5 //+0.5 //sebelum 1

        sudutX2=temp_sudutX2-10*sin(sudutNgurang*3.14/360);
        sudutY2=temp_sudutY2;//-2*cos(sudutNgurang*3.14/360);//-3
        sudutZ2=temp_sudutZ2;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2))//0
        speed=200;//+50*cos(sudutNgurang*3.14/360);
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
}	
	usleep(50000);

        f2=0;
*/
         //cout<<"Masukkan sudut X: ";
								        // BALIKIN MIRING //
 	//for(int i=0;i<=38;i++)
        while(f2<=90){
	f2=f2+0.08; //0.08
        sudutNgurang=f2;
        tempx=x;//-sin(sudutNgurang*3.14/180)*2;
        tempy=y-y*sin(sudutNgurang*3.14/180);//2.4
        tempz=z;

        temp_sudutX=sudutX-sudutX*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        temp_sudutY=sudutY-1*sin(sudutNgurang*3.14/180);//+2*sin(sudutNgurang*3.14/180);
        temp_sudutZ=sudutZ;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));

        tempx2=x2;//+sin(sudutNgurang*3.14/180)*2;
        tempy2=y2-y2*sin(sudutNgurang*3.14/180);//2.4
        tempz2=z2;//0.2;//.2;

        temp_sudutX2=sudutX2-sudutX2*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2)); +4*sin
        temp_sudutY2=sudutY2-1*sin(sudutNgurang*3.14/180);//+2*sin(sudutNgurang*3.14/180);
        temp_sudutZ2=sudutZ2;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2
        speed=50;//cos(sudutNgurang*3.14/360);

//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
         inverseAndre(temp_sudutX,temp_sudutY,temp_sudutZ,tempx,tempy,tempz,temp_sudutX2,temp_sudutY2,temp_sudutZ2,tempx2,tempy2,tempz2,speed,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
        //usleep(1000);
//       cout<<"Masukkan sudut X: ";
                }
        f2=0;
for (int i=0;i<4;i++){

         cout<<"Masukkan sudut X: "<<sudutX2<<endl;
        usleep(500000); //usleep(500000);
	//sleep(1);//2

							        // MIRING KIRI//
        while(f2<=90)

        {f2=f2+0.08;//0.08
        sudutNgurang=f2;
        x=tempx;//+sin(sudutNgurang*3.14/180)*2;
        y=tempy+4*sin(sudutNgurang*3.14/180);//0.5 sebelum
        z=tempz;//0.5;//cos(sudutNgurang*3.14/180);

        sudutX=temp_sudutX+2*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));   	-0.5
 	sudutY=temp_sudutY-2*sin(sudutNgurang*3.14/180);
        sudutZ=temp_sudutZ;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=tempx2;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=tempy2+1*sin(sudutNgurang*3.14/180);
        z2=tempz2;//0.2;//cos(sudutNgurang*3.14/180);

        sudutX2=temp_sudutX2-8*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        sudutY2=temp_sudutY2-2*sin(sudutNgurang*3.14/180);
        sudutZ2=temp_sudutZ2;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=150;//cos(sudutNgurang*3.14/360);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,522,502,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
                        }
        f2=0;
        usleep(60000);
						        //ANGKAT KAKI KANAN DAN MENDARAT/
        //======================
        outdx = 0;
        outdy = 0;
        edysblm = 0;
        edxsblm = 0;
        pidx = 0;
        pidy= 0;
        //==================
        while(f2<=180){
        f2=f2+0.08; //0.08
        sudutNgurang=f2;
        tempx=x+0*sin(sudutNgurang*3.14/180);
        tempy=y+0*sin(sudutNgurang*3.14/180);//1+1*sin(sudutNgurang*3.14/180);//- pidy;//2.4	//sebelum +1	
        tempz=z*cos(sudutNgurang*3.14/180); //sebelum 2		//2

        temp_sudutX=sudutX;//+10*sin(sudutNgurang*3.14/180);//- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/ 	//15 sebelum b
        temp_sudutY=sudutY;//-4*sin(2*sudutNgurang*3.14/180);//+0*sin(sudutNgurang*3.14/180);//+sin(sudutNgurang*3.14/180);
        temp_sudutZ=sudutZ;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        tempx2=x2+0*sin(sudutNgurang*3.14/180);
        tempy2=y2+0*sin(sudutNgurang*3.14/180);// -1
        tempz2=z2*cos(sudutNgurang*3.14/180); 			//-1
 	//z2=-1*cos(sudutNgurang*3.14/180);

        temp_sudutX2=sudutX2;//+0*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        temp_sudutY2=sudutY;//-4*sin(2*sudutNgurang*3.14/180);//+2*sin(2*sudutNgurang*3.14/360);
        temp_sudutZ2=sudutZ2;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=150;//+50*cos(sudutNgurang*3.14/360);
	cout<<temp_sudutX<<endl<<temp_sudutY<<endl<<temp_sudutZ<<endl<<tempx
	<<endl<<tempy<<endl<<tempz<<endl;
 	inverseAndre(temp_sudutX,temp_sudutY,temp_sudutZ,tempx,tempy,tempz,temp_sudutX2,temp_sudutY2,temp_sudutZ2,tempx2,tempy2,tempz2,speed,522,502,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
	//cout<<endl<<cek_move()<<endl<<endl;
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
//		usleep(500);    
    			//cout<<x<<endl;
	}
        usleep(60000);
        f2=0;
						        // BALIKIN MIRING//

        while(f2<=90){
        f2=f2+0.08;
        sudutNgurang=f2;
        x=tempx;//+sin(sudutNgurang*3.14/180)*1.4;
        y=tempy-tempy*sin(sudutNgurang*3.14/180);//2.4
        z=tempz;//-3*sin(sudutNgurang*3.14/360);//cos(sudutNgurang*3.14/180);

        sudutX=temp_sudutX-temp_sudutX*sin(sudutNgurang*3.14/180);
        sudutY=temp_sudutY;//-1*sin(sudutNgurang*3.14/180);//-1*sin(sudutNgurang*3.14/180);
        sudutZ=temp_sudutZ;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=tempx2;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=tempy2-tempy2*sin(sudutNgurang*3.14/180);//2.4
        z2=tempz2;//+1*sin(sudutNgurang*3.14/180);//-0.5;//-0.5;//cos(sudutNgurang*3.14/180);

        sudutX2=temp_sudutX2*sin(sudutNgurang*3.14/180);
        sudutY2=temp_sudutY2;//-1*sin(sudutNgurang*3.14/180);//-1*sin(sudutNgurang*3.14/180);
        sudutZ2=temp_sudutZ2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=50;//cos(sudutNgurang*3.14/360);

//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,522,502,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);

                }
        f2=0;
	cout<<x<<endl<<y<<endl<<z<<endl<<sudutX<<endl<<sudutY<<endl<<sudutZ<<endl;
	cout<<x2<<endl<<y2<<endl<<z2<<endl<<sudutX2<<endl<<sudutY2<<endl<<sudutZ2<<endl;

                //usleep(500000);
        usleep(1000000);//usleep(300000); //usleep(300000)
							        // MIRING KANAN //
while(f2<=90)

        {f2=f2+0.08;
        sudutNgurang=f2;
        tempx=x;//+sin(sudutNgurang*3.14/180)*2;
        tempy=y-1*sin(sudutNgurang*3.14/180);//2.4
        tempz=z;//0.5;//cos(sudutNgurang*3.14/180);

        temp_sudutX=sudutX-10*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        temp_sudutY=sudutY+3*sin(sudutNgurang*3.14/180);
        temp_sudutZ=sudutZ;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));

        tempx2=x2;//-sin(sudutNgurang*3.14/180)*2.4;
        tempy2=y2-4.5*sin(sudutNgurang*3.14/180);//2.4
        tempz2=z2;//0.2;//cos(sudutNgurang*3.14/180);

        temp_sudutX2=sudutX2+2*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        temp_sudutY2=sudutY2+3*sin(sudutNgurang*3.14/180);
        temp_sudutZ2=sudutZ2;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=50;//cos(sudutNgurang*3.14/360);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	 inverseAndre(temp_sudutX,temp_sudutY,temp_sudutZ,tempx,tempy,tempz,temp_sudutX2,temp_sudutY2,temp_sudutZ2,tempx2,tempy2,tempz2,speed,522,502,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);                        }
        usleep(50000);

				//angkat kaki kiri dan mendarat
	f2=0;
	while(f2<=180){
        f2=f2+0.08;
        sudutNgurang=f2;
        x=tempx+0*sin(sudutNgurang*3.14/180);
        y=tempy-0;//sin(sudutNgurang*3.14/180);//1+1*sin(sudutNgurang*3.14/180);//- pidy;//2.4
        z=tempz*cos(sudutNgurang*3.14/180);//sebelum -1

        sudutX=temp_sudutX;//-2*sin(sudutNgurang*3.14/180);//- pidx;//(pow(sin(sudutNgurang*3.14/180),2));
        sudutY=temp_sudutY;//sin(2*sudutNgurang*3.14/360);//+sin(sudutNgurang*3.14/180);
        sudutZ=temp_sudutZ;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=tempx2+0*sin(sudutNgurang*3.14/180);
        y2=tempy2-0*sin(sudutNgurang*3.14/180);//2.4
        z2=tempz2*cos(sudutNgurang*3.14/180); 			//2
        //z2=-1*cos(sudutNgurang*3.14/180);

        sudutX2=temp_sudutX2-0*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        sudutY2=temp_sudutY2;//+2*sin(2*sudutNgurang*3.14/360);
        sudutZ2=temp_sudutZ2;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//+50*cos(sudutNgurang*3.14/360);
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,522,502,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');

                                                        }

	usleep(5000);
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
        inverseAndre(temp_sudutX,temp_sudutY,temp_sudutZ,tempx,tempy,tempz,temp_sudutX2,temp_sudutY2,temp_sudutZ2,tempx2,tempy2,tempz2,speed,522,502,'2',480,520,270,750,270,750,data_image.x_head,data_image.y_head);

        }
//	cout<<"y :"<<y<<" y2: "<<y2<<endl;
	cout<<tempz<<endl<<tempz2<<endl;
	//tempz=1;tempz2=-1;
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

int main(){
	int fd;
	dxl_initialize(0,1);
//        dxl_write_word(19,32,800);
//        dxl_write_word(20,32,800);
	data_image.y_head=310;
	data_image.x_head=530;
	data_image.condition=1;	
	berdiri();
	sleep(2);
	//sleep(10);
//	char * myfifo="/tmp/myfifo";
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
			cout<<"asu"<<endl;
			jalan();
			berdiri();
			sleep(2);
			break;
		}
	}
	close(fd);
	return 0;
}

