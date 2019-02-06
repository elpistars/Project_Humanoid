#include "/home/pi/systemTest/library/allLibtune.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <math.h>
#include <chrono>
#include <Kalman.h>
#include <mpu.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#define RAD_TO_DEG 180/M_PI
#define MPU_X_SP -3.4
#define MPU_Y_SP -23
#define K_Y_OFFSET 0.1
#define K_Z_OFFSET 0.5
double px,dx,ix,kpx,kdx,kix,errorx,sigmerx,errorxsem,sigsemx,pidx;
double py,dy,iy,kpy,kdy,kiy,errory,sigmery,errorysem,sigsemy,pidy;
double ang,sudutngurang,f2;
//Kalman Object
Kalman kalmanX;
Kalman kalmanY;

//PID class
class PID {
private:
  double p_error;
  double i_error;
  double d_error;

  double Kp;
  double Ki;
  double Kd;
public:
  PID(){};
  void Init(double kp, double ki, double kd);
  void UpdateError(double cte);
  double getValue();
};
void PID::Init(double kp, double ki, double kd){
    Kp=kp;
    Ki=ki;
    Kd=kd;
    p_error=0; i_error=0; d_error=0;
  }
void PID::UpdateError(double cte){
    double pre_cte=p_error;
    p_error=cte;
    i_error+=cte;
    d_error=cte-pre_cte;
    //std::cout<<"p: "<<Kp*p_error<<"__i: "<<Ki*i_error<<"__d: "<<Kd*d_error<<std::endl;
  }
double PID::getValue(){
  return Kp*p_error + Ki*i_error + Kd*d_error;
}

//MPU function
void get_mpu(double &kalAngleX, double &kalAngleY);

//variable timer
auto t_start=std::chrono::high_resolution_clock::now();
auto t_end=std::chrono::high_resolution_clock::now();
std::chrono::duration<double> t_span;

//variable Kalman
double roll,pitch,kalX,kalY;

//variable invers kinematik
double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;

struct {
        int condition,x_head,y_head;
}data_image;
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
inline void berdiri(double kalmanx, double kalmany){
	data_image.x_head=558;
	data_image.y_head=310;
        int speed, com7, com8;
//       com7=4;
  //      com8=4;
        dxl_write_word(10,26,1);
        dxl_write_word(10,27,1);
        dxl_write_word(9,26,1);
        dxl_write_word(9,27,1);
	if (f2>180){
          f2=0;}
	
        //kirimPacketTorque(12,id,tor);
        //for(int i=0;i<=11;i++){
          dxl_write_word(10,28,4);
          dxl_write_word(10,29,4);
          dxl_write_word(9,28,4);
          dxl_write_word(9,29,4);
	f2=f2+4;

	sudutngurang=f2;

        x=-10.8;//2*/sin(sudutngurang*3.14/180);// tinggi kaki kanan //-13.4
        y=+K_Y_OFFSET*kalmanx;// miring kaki
        z=0.3-K_Z_OFFSET*kalmany;// langkah kaki

	sudutX=0+kalmanx;
	sudutY=23-kalmany;
        sudutZ=0;//putaran badan
        if (sudutX>20){
          sudutX=20;}
        else if (sudutX<-20){
	  sudutX=-20;}
        if (y>3.5){
	  y=3.5;}
        else if (y<-3.5){
	  y=-3.5;}
	if (sudutY>35){
	  sudutY=35;}
	else if(sudutY<-35){
	  sudutY=-35;}
	if(z>2){
	  z=2;}
	else if(z<-2){
	  z=-2;}

        x2=-10.8;//0.1*ang; //tinggi kaki kiri
        y2=0+K_Y_OFFSET*kalmanx;
        z2=0+K_Z_OFFSET*kalmany;
	if(y2>3.5){
	  y2=3.5;}
	else if(y2<-3.5){
	  y2=-3.5;}
	if(z2>2){
	  z2=2;}
	else if(z2<-2){
	  z2=-2;}
	sudutX2=sudutX*(-1);
        sudutY2=sudutY;
        sudutZ2=0;
	//std::cout<<y<<"   "<<y2<<std::endl;
        //std::cout<<f2<<","<<py<<"  |  "<<sudutY2<<","<<z2<<std::endl;
    inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,200+kalmanx+kalmany,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);//250,780,500,500
	usleep(10000);
}
int main(int argc,char*argv[]){
/*  struct k_pid{
    double p,i,d;
  }kpid_x,kpid_y;*/
  double x_offset, y_offset,sp_x,sp_y;
  if (argc<9){
    std::cout<<"You must input Kp, Ki, Kd, SP"<<std::endl;
    return 1;
  }
  dxl_initialize(0,1);
  int fd;
//  char *myfifo="/tmp/myfifo";
 // mkfifo(myfifo,0666);
  std::istringstream s1(argv[1]);
  std::istringstream s2(argv[2]);
  std::istringstream s3(argv[3]);
  std::istringstream s4(argv[4]);
  std::istringstream s5(argv[5]);
  std::istringstream s6(argv[6]);
  std::istringstream s7(argv[7]);
  std::istringstream s8(argv[8]);
  s1 >> kpx;
  s2 >> kix;
  s3 >> kdx;
  s4 >> kpy;
  s5 >> kiy;
  s6 >> kdy;
  s7 >> sp_x;
  s8 >> sp_y;
  /*kpid_x.p=0.2;
  kpid_x.i=0;
  kpid_x.d=0.02;
  kpid_y.p=0.03;
  kpid_y.i=0;
  kpid_y.d=0.02;*/
	 px=0;
	dx=0;
	ix=0;
//	kpx=0.3;
//	kdx=0.12;
//	kix=0;
	errorx=0;
	sigmerx=0;
	errorxsem=0;
	sigsemx=0;
	
	 py=0;
        dy=0;
        iy=0;
        //kpy=0.13;
       // kdy=0.12;
       // kiy=0;
        errory=0;
        sigmery=0;
        errorysem=0;
        sigsemy=0;


//  sp_x=0;
//  sp_y=-13;
  berdiri(0,0);
  InitMPU();
  std::cout<<"K pid X: "<<kpx<<" "<<kix<<" "<<kdx<<std::endl;
  std::cout<<"K pid X: "<<kpy<<" "<<kiy<<" "<<kdy<<std::endl;
  std::cout<<"SP: "<<sp_x<<" "<<sp_y<<std::endl;

  while(std::cin.get() != '\n');
  /*PID pid_mpu_x;
  PID pid_mpu_y;
  pid_mpu_x.Init(kpx,kix,kdx);
  pid_mpu_y.Init(kpy,kiy,kdy);*/
  accel_read();
  roll= atan(accel.y / sqrt(accel.x * accel.x + accel.z * accel.z)) * RAD_TO_DEG;
  pitch= atan2(-accel.x, accel.z) * RAD_TO_DEG;
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyro.x=roll;
  gyro.y=pitch;
  t_start = std::chrono::high_resolution_clock::now();
  while(1){
/*    if(fd=open(myfifo, O_RDONLY)<0)
      std::cout<<"Error when opening fifo\n";
    //std::cout<<fd;
      read(fd,&data_mpu,sizeof(data_mpu));
     std::cout<<data_mpu.x<<" | "<<data_mpu.y<<std::endl;
      //close(fd);
*/
    get_mpu(kalX,kalY);
/*  pid_mpu_x.UpdateError(kalX-sp_x);
    pid_mpu_y.UpdateError(kalY-sp_y);
    pidx=pid_mpu_x.getValue();
    pidy=pid_mpu_y.getValue();*/
	errorx=kalX-sp_x;
	sigmerx=errorx+sigsemx;
	px=kpx*errorx;
	dx=kdx*(errorx-errorxsem);
	errorxsem=errorx;
	ix=kix*(sigmerx);
	sigmerx=sigsemx;
	pidx=px+dx+ix;

 	 errory=kalY-sp_y;
        sigmery=errory+sigsemy;
        py=kpy*errory;
        dy=kdy*(errory-errorysem);
        errorysem=errory;
        iy=kiy*(sigmery);
        sigmery=sigsemy;
        pidy=py+dy+iy;

ang=sp_x;
    berdiri(pidx,pidy);//y_offset);
  }

}
