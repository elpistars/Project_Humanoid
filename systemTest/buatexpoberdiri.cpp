#include "/home/pi/systemTest/library/allLibtune.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"
    	int tanganKiri;
        int tangan2Kiri;
        int tangan3Kiri;
        int tanganKanan;
        int tangan2Kanan;
        int tangan3Kanan;

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
double a=0.08;
struct {
	int x_head;
	int y_head;
}data_image;
void verse(){
	data_image.x_head=512;
	data_image.y_head=512;
        double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
        int speed,  com7, com8;


/*	dxl_write_word(10,27,1);
        dxl_write_word(9,26,1);
        dxl_write_word(9,27,1);

        //kirimPacketTorque(12,id,tor);
        //for(int i=0;i<=11;i++){
                dxl_write_word(10,28,4);
          dxl_write_word(10,29,4);
  dxl_write_word(9,28,4);
  dxl_write_word(9,29,4);
*/	
	x=-13.4;// tinggi kaki //-13.4
        y=0;// miring kaki
        z=0.5;// langkah kaki //0

        sudutX=0;//miring badan
        sudutY=14;//kebungkukan badan
        sudutZ=0;//putaran badan

        x2=-13.4;
        y2=0;//3;
        z2=0.5;

        sudutX2=0;//-6;
        sudutY2=14;
        sudutZ2=0;

  //      dxl_write_word(3,30,300);

//	  dxl_write_word(4,30,724);
        // inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,$
 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,200,512,512,'2',500,500,250,780,500,500,data_image.x_head,data_image.y_head);

        cout<<"Masukkan Speed: ";
        //cin>>speed;
        sleep(1);

                //kirimPacketInverse(x,y);

        //for(int i=1;i<=10;i++){
        // MIRING //

	 for(int i=1; i<=50; i++){
                        tanganKiri=500-200*sin(i*3.14*0.01);//500-200*sin
                        tanganKanan=500+200*sin(i*3.14*0.01);//500+200*sin
                        //tanganKiri=700;
			tangan2Kanan=250+250*sin(i*3.14*0.01);//250+250*sin
			tangan2Kiri=780-280*sin(i*3.14*0.01);//780-280*sin
                        cout << tanganKiri << endl;
                      //  int data1[][18] = {tanganKiri, 500, 700, tanganKanan, 500, 300};

                        //kirimPacketGerak(6, 1, 1, id, data1, 1, delay1, speed1);
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,512,512,'2',tanganKanan,tanganKiri,tangan2Kanan,tangan2Kiri,tanganKiri,tanganKanan,data_image.x_head,data_image.y_head);
                        usleep(10000);
                }
	usleep(20000);
        f2=0;
	 while(f2<=90.0)
         //for(int i=0;i<=38;i++)
                {
		f2=f2+0.08;
        //cout<<"Masukkan X: ";
         sudutNgurang=f2;
        x=-13.4;//-2*sin(sudutNgurang*3.14/180)*2;
        y=0;//-1*sin(sudutNgurang*3.14/180);//-0.5
        z=0;
        sudutX=0;//2*sin(sudutNgurang*3.14/180);
        sudutY=14;//+sin(sudutNgurang*3.14/180);
        sudutZ=512-50*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-13.4;//+sin(sudutNgurang*3.14/180)*2;
        y2=0;//-1*sin(sudutNgurang*3.14/180);//2.4
        z2=0;
        sudutX2=-0;//2*sin(sudutNgurang*3.14/180);
        sudutY2=14;//-sin(sudutNgurang*3.14/180);
        sudutZ2=512-50*(pow(sin(sudutNgurang*3.14/180),2));
          speed=100;//cos(sudutNgurang*3.14/360);
                        tangan3Kiri=700-abs(200*sin(sudutNgurang*3.14/90)) ;
                        tangan3Kanan=300+abs(200*sin(sudutNgurang*3.14/90));
                        cout << tanganKiri << endl;
                        int delay1[] = {1000, 1000};
                        int speed1[] = {100, 100};
        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z$
			inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,100,sudutZ,sudutZ2,'2',700,300,500,500,tangan3Kanan,tangan3Kiri,data_image.x_head,data_image.y_head);
			cout<<sudutZ<<endl;
     			usleep(300);

//         cout<<"Masukkan sudut X: "<<f2<<endl;
	
        }
f2=0;
   while(f2<=90.0)
         //for(int i=0;i<=38;i++)
                {
		f2=f2+a;
        //cout<<"Masukkan X: ";
         sudutNgurang=f2;
        x=-13.4;//-2*sin(sudutNgurang*3.14/180)*2;
        y=0;//-1*sin(sudutNgurang*3.14/180);//
        z=0;
        sudutX=0;//2*sin(sudutNgurang*3.14/180);
        sudutY=14;//+sin(sudutNgurang*3.14/180);
        sudutZ=462+50*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-13.4;//+sin(sudutNgurang*3.14/180)*2;
        y2=0;//-1*sin(sudutNgurang*3.14/180);//2.4
        z2=0;
       sudutX2=-0;//2*sin(sudutNgurang*3.14/180);
        sudutY2=14;//-sin(sudutNgurang*3.14/180);
        sudutZ2=462+50*(pow(sin(sudutNgurang*3.14/180),2));
          speed=100;//cos(sudutNgurang*3.14/360);

        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z$
  			tangan3Kiri=700-abs(200*sin(sudutNgurang*3.14/90)) ;
                        tangan3Kanan=300+abs(200*sin(sudutNgurang*3.14/90));
                        cout << tanganKiri << endl;
                        int delay1[] = {1000, 1000};
                        int speed1[] = {100, 100};
        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z$
                        inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,100,sudutZ,sudutZ2,'2',7000,300,500,500,tangan3Kanan,tangan3Kiri,data_image.x_head,data_image.y_head);
                        cout<<sudutZ<<endl;
                        usleep(300);

        cout<<sudutZ<<endl;

//      usleep(1000);
//         cout<<"Masukkan sudut X: "<<f2<<endl;
        }
f2=0;
  while(f2<=90.0)
         //for(int i=0;i<=38;i++)
                {
		f2=f2+a;
        //cout<<"Masukkan X: ";
         sudutNgurang=f2;
        x=-13.4;//-2*sin(sudutNgurang*3.14/180)*2;
        y=0;//-1*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=0;//2*sin(sudutNgurang*3.14/180);
        sudutY=14;//+sin(sudutNgurang*3.14/180);
        sudutZ=512+50*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-13.4;//+sin(sudutNgurang*3.14/180)*2;
        y2=0;//-1*sin(sudutNgurang*3.14/180);//2.4
        z2=0;
       sudutX2=-0;//2*sin(sudutNgurang*3.14/180);
        sudutY2=14;//-sin(sudutNgurang*3.14/180);
        sudutZ2=512+50*(pow(sin(sudutNgurang*3.14/180),2));
          speed=100;//cos(sudutNgurang*3.14/360);

        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z$
			  tangan3Kiri=700-abs(200*sin(sudutNgurang*3.14/90)) ;
                        tangan3Kanan=300+abs(200*sin(sudutNgurang*3.14/90));
                        cout << tanganKiri << endl;
                        int delay1[] = {1000, 1000};
                        int speed1[] = {100, 100};
        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z$
                        inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,100,sudutZ,sudutZ2,'2',700,300,500,500,tangan3Kanan,tangan3Kiri,data_image.x_head,data_image.y_head);
                        cout<<sudutZ<<endl;
                        usleep(300);

        cout<<sudutZ<<endl;

}

f2=0;
  while(f2<=90.0)
         //for(int i=0;i<=38;i++)
                {f2=f2+a;
        //cout<<"Masukkan X: ";
         sudutNgurang=f2;
        x=-13.4;//-2*sin(sudutNgurang*3.14/180)*2;
        y=0;//-1*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=0;//2*sin(sudutNgurang*3.14/180);
        sudutY=14;//+sin(sudutNgurang*3.14/180);
        sudutZ=562-50*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-13.4;//+sin(sudutNgurang*3.14/180)*2;
        y2=0;//-1*sin(sudutNgurang*3.14/180);//2.4
        z2=0;
       sudutX2=-0;//2*sin(sudutNgurang*3.14/180);
        sudutY2=14;//-sin(sudutNgurang*3.14/180);
        sudutZ2=562-50*(pow(sin(sudutNgurang*3.14/180),2));
          speed=100;//cos(sudutNgurang*3.14/360);

        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z$
  			tangan3Kiri=700-abs(200*sin(sudutNgurang*3.14/90)) ;
                        tangan3Kanan=300+abs(200*sin(sudutNgurang*3.14/90));
                        cout << tanganKiri << endl;
                        int delay1[] = {1000, 1000};
                        int speed1[] = {100, 100};
        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z$
                        inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,100,sudutZ,sudutZ2,'2',700,300,500,500,tangan3Kanan,tangan3Kiri,data_image.x_head,data_image.y_head);
                        cout<<sudutZ<<endl;
                        usleep(300);

        cout<<sudutZ<<endl;

}
f2=0;
  while(f2<=90.0)
         //for(int i=0;i<=38;i++)
                {f2=f2+a;
        //cout<<"Masukkan X: ";
         sudutNgurang=f2;
        x=-13.4;//-2*sin(sudutNgurang*3.14/180)*2;
        y=0;//-1*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=0;//2*sin(sudutNgurang*3.14/180);
        sudutY=14;//+sin(sudutNgurang*3.14/180);
        sudutZ=512;//-50*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-13.4;//+sin(sudutNgurang*3.14/180)*2;
        y2=0;//-1*sin(sudutNgurang*3.14/180);//2.4
        z2=0;
        sudutX2=-0;//2*sin(sudutNgurang*3.14/180);
        sudutY2=14;//-sin(sudutNgurang*3.14/180);
        sudutZ2=512;//-50*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);
	//tanganKiri=700;
        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z$
  			tangan3Kiri=700-abs(200*sin(sudutNgurang*3.14/180)) ;
                        tangan3Kanan=300+abs(200*sin(sudutNgurang*3.14/180));
                        tangan2Kanan=500-250*abs(sin(sudutNgurang*3.14/180));
                        tangan2Kiri=500+280*abs(sin(sudutNgurang*3.14/180));

			 cout << tanganKiri << endl;
                        int delay1[] = {1000, 1000};
                        int speed1[] = {100, 100};
        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z$
                        inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,100,sudutZ,sudutZ2,'2',tangan3Kanan,tangan3Kiri,tangan2Kanan,tangan2Kiri,tangan3Kanan,tangan3Kiri,data_image.x_head,data_image.y_head);
                        cout<<sudutZ<<endl;
                        usleep(300);

        cout<<sudutZ<<endl;

}



}



int main(){

if(dxl_initialize(0,1)==0){
        cout<<"Error connecting"<<endl;
        return 0;
    }

else{
                cout<<"connect success!"<<endl;
        }

while(1){

        verse();
//usleep(3000);
sleep(1);
}
        return 0;
}
