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
double f2;
double a=0.02;
struct {
	int x_head;
	int y_head;
}data_image;

void verse(){

        double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2;
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
 ^        cout<<"Masukkan sudut Y: ";
        cin>>sudutY;
        cout<<"Masukkan sudut Z: ";
        cin>>sudutZ;

        cout<<"Masukkan X: ";
        cin>>x;
        cout<<"Masukkan Y: ";
        cin>>y;
        cout<<"Masukkan Z: ";
        cin>>z;*/
        x=-11.4;//tinggi kaki kanan //-13.4 bener  //berdiri -9
        y=0;// miring kaki// 0 bener
        z=0;// langkah kaki //2.5 berdiri        //2

        sudutX=2;//miring badan
        sudutY=17;//kebungkukan badan // -3 berdiri    //2.5
        sudutZ=0;//putaran badan

        x2=-14.4; // tinggi kaki kiri //-13 berdiri 
        y2=4;//; // 0 berdiri                  //3 
        z2=0; // 2.5 berdiri                    //4

        sudutX2=0;//                        //9
        sudutY2=20; // -3 berdiri           //2.5
        sudutZ2=0;

        dxl_write_word(3,30,300);
        dxl_write_word(4,30,724);
        // inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y$
         inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);
        cout<<"Masukkan Speed: ";
        //cin>>speed;
        sleep(7);}
int main(){
if(dxl_initialize(0,1)==0){
        cout<<"Error connecting"<<endl;
        return 0;
    }

else{
                cout<<"connect success!"<<endl;
        }


while (1){  
	verse();
}
}
