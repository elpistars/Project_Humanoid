#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <mpu.h>
#include <Kalman.h>
#include <chrono>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#define RAD_TO_DEG 180/M_PI

Kalman kalmanX;
Kalman kalmanY;
struct data_mpu{
  double x,y;
}dat_mpu;
int fd;
int main(){
  double roll,pitch;
  double kalAngleX,kalAngleY;
  char *myfifo ="/tmp/myfifo";
  mkfifo(myfifo,0666);
  std::chrono::duration<double> t_span;
  double dt;
  auto t_start=std::chrono::high_resolution_clock::now();
  auto t_end=std::chrono::high_resolution_clock::now();
  InitMPU();
  sleep(1);
  accel_read();
  roll= atan(accel.y / sqrt(accel.x * accel.x + accel.z * accel.z)) * RAD_TO_DEG;
  pitch= atan2(-accel.x, accel.z) * RAD_TO_DEG;
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyro.x=roll;
  gyro.y=pitch;
  t_start = std::chrono::high_resolution_clock::now();

  while(1){
/*    if((fd=open(myfifo,O_WRONLY))<0){
      std::cout<<"Error when opening fifo\n";
    }*/
    gyro_read();
    accel_read();
    t_end = std::chrono::high_resolution_clock::now();
    t_span=std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
    dt=t_span.count();
    t_start = std::chrono::high_resolution_clock::now();
    roll= atan(accel.y / sqrt(accel.x * accel.x + accel.z * accel.z)) * RAD_TO_DEG;
    pitch= atan2(-accel.x, accel.z) * RAD_TO_DEG;
    if((pitch < -90 && kalAngleY > 90) || (roll > 90 &&kalAngleX < -90)){
      kalmanY.setAngle(pitch);
      kalAngleY = pitch;
    }
    else{
      kalAngleY = kalmanY.getAngle(pitch,gyro.y,dt);
    }
    if(abs(kalAngleY) > 90)
      gyro.x=-gyro.x;
    kalAngleX = kalmanX.getAngle(roll, gyro.x, dt);
    dat_mpu.x=kalAngleX;
    dat_mpu.y=kalAngleY;
    std::cout<<std::setprecision(3)<<dat_mpu.x<<" || "<<dat_mpu.y<<" || "<<dt<<std::endl<<std::endl;
    /*write(fd, &dat_mpu, sizeof(dat_mpu));
    close(fd);*/
    usleep(48000);
  }
  close(fd);
  close(file_i2c);
  return 0;
}
