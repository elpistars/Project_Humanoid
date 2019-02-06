#include "/home/pi/systemTest/library/allLibtune.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

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


void verse(){

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

	x=-9.4;// tinggi kaki
        y=0;// miring kaki
        z=0.5;// langkah kaki
        sudutX=0;//miring badan
        sudutY=-3;//kebungkukan badan
        sudutZ=0;//putaran badan
        x2=-9.4;
        y2=0;//3;
        z2=0;
        sudutX2=0;//-6;
        sudutY2=-3;
        sudutZ2=0;
        dxl_write_word(3,30,300);
        dxl_write_word(4,30,724);
        // inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
         inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,512,512,'2');
        cout<<"Masukkan Speed: ";
        //cin>>speed;
        sleep(3);
	



	 sudutNgurang=2.4*i;
        x=-9.4;//-2*sin(sudutNgurang*3.14/180)*2;
        y=-1;//sin(sudutNgurang*3.14/180);//2.4
        z=0.5;
        sudutX=1;//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=-3;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-9.4;//+sin(sudutNgurang*3.14/180)*2;
        y2=-1;//sin(sudutNgurang*3.14/180);//2.4
        z2=0;
        sudutX2=-1;//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=-3;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
          speed=300;//cos(sudutNgurang*3.14/360);

        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
         inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
        usleep(1000);

	}


int main(){

if(dxl_initialize(0,1)==0){
        cout<<"Error connecting"<<endl;
        return 0;
    }

else{
                cout<<"connect success!"<<endl;
        }
	
  verse();
}
