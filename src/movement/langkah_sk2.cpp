#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h> 
#include <string.h>


#include "movement/foot_ik.h" 

#include "putarkanan.cpp"
#include "putarkiri.cpp"
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
        r_ik.x=-12.4;// tinggi kaki kanan //-13.4
        r_ik.y=0;// miring kaki
        r_ik.z=0.5;// langkah kaki	//0.3

        r_ik.sudutX=0;//miring badan
        r_ik.sudutY=24;//kebungkukan badan		//22
        r_ik.sudutZ=0;//putaran badan

        l_ik.x=-12; //tinggi kaki kiri
        l_ik.y=0;
        l_ik.z=-0.3;

        l_sudutX2=0;
        l_sudutY2=24;
        l_sudutZ2=0;
        /*cout<<sudutX<<endl<<sudutY<<endl<<sudutZ<<endl<<x
        <<endl<<y<<endl<<z<<endl;*/
        // inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);//250,780,500,500
        //cout<<"Masukkan Speed: ";
        //cin>>speed;
        sleep(1);//5
         //kirimPacketInverse(x,y);
        //cin>>speed;
        //for(int i=1;i<=10;i++){
		}
		
void jalan(){
        int speed, com7, com8;
        x=-12.4;// tinggi kaki kanan //-13.4
        y=0;// miring kaki
        z=0.5;// langkah kaki   //0.3

        sudutX=0;//miring badan
        sudutY=24;//kebungkukan badan           //22
        sudutZ=0;//putaran badan

        x2=-12; //tinggi kaki kiri
        y2=0;
        z2=-0.3;

        sudutX2=0;
        sudutY2=24;
        sudutZ2=0;

	tempx=-12.4;// tinggi kaki kanan //-13.4
        tempy=0;// miring kaki
        tempz=0.5;// langkah kaki 

        temp_sudutX=0;//miring badan
        temp_sudutY=24;//kebungkukan badan 
        temp_sudutZ=0;//putaran badan

        tempx2=-12; //tinggi kaki kiri
        tempy2=0;
        tempz2=-0.3;

        temp_sudutX2=0;
        temp_sudutY2=24; 
        temp_sudutZ2=0;
 			//-----------------------------------------------------------------							        // MIRING KANAN//
        f2=0;
	while(f2<=90.0)
         //for(int i=0;i<=38;i++)
                {f2=f2+0.008;
        //cout<<"Masukkan X: ";
         sudutNgurang=f2;
        tempx=x;//-2*sin(sudutNgurang*3.14/180)*2;
        tempy=y-1.5*sin(sudutNgurang*3.14/180);//
        tempz=z; //2.5

        temp_sudutX=sudutX-4*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        temp_sudutY=sudutY+2*sin(sudutNgurang*3.14/180);//+sin(sudutNgurang*3.14/180);
        temp_sudutZ=sudutZ;//3*(pow(sin(sudutNgurang*3.14/180),2));

        tempx2=x2;//+sin(sudutNgurang*3.14/180)*2;
        tempy2=y2-3*sin(sudutNgurang*3.14/180);//-1.5
        tempz2=z2;

        temp_sudutX2=sudutX2-8*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        temp_sudutY2=sudutY2+2*sin(sudutNgurang*3.14/180);//-sin(sudutNgurang*3.14/180);
        temp_sudutZ2=sudutZ2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);
        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
         inverseAndre(temp_sudutX,temp_sudutY,temp_sudutZ,tempx,tempy,tempz,temp_sudutX2,temp_sudutY2,temp_sudutZ2,tempx2,tempy2,tempz2,speed,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
//      usleep(1000);
//         cout<<"Masukkan sudut X: "<<f2<<endl;
        }

        f2=0;
	//sleep(5); 
	
	usleep(50000);
       							 //  ANGKAT KAKI KIRI DAN MENDARAT //
 	//cout<<"Masukkan sudut X: ";
         //for(int i=0;i<=76;i++)
   	 while(f2<=180)
	{f2=f2+0.03; //0.05

	sudutNgurang=f2;
        x=tempx+0*sin(sudutNgurang*3.14/180); //+2	//
        y=tempy-2*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//		-1
        z=tempz-0.5*sin(sudutNgurang*3.14/360); //0+0.5-0.5*cos(sudutNgurang*3.14/180);	//  =0.5*sin(360)

        sudutX=temp_sudutX;//-8*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));//sebelum -4
        sudutY=temp_sudutY;//-0*sin(sudutNgurang*3.14/360);
        sudutZ=temp_sudutZ;//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=tempx2+2*sin(sudutNgurang*3.14/180); 
        y2=tempy2-0*sin(sudutNgurang*3.14/180);// + pidy;//sin(sudutNgurang*3.14/180);//2.5+2	// -1
        z2=tempz2+0.5*sin(sudutNgurang*3.14/360);//-0.5 //+0.5 //  2*sin(360)

        sudutX2=temp_sudutX2-4*sin(sudutNgurang*3.14/180);// - pidx;(pow(sin(sudutNgurang*3.14/180),2));		//-20 BENER
        sudutY2=temp_sudutY2;//-0*sin(sudutNgurang*3.14/360);//-3
    	sudutZ2=temp_sudutZ2;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2))//0
        speed=150;//+50*cos(sudutNgurang*3.14/360);

        //inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        //baca sensor

	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);

//sensen();
//              if(i>46){
                        //PID x
                        // P x
                              // P y
}	
	f2=0;
	//usleep(500);
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
	f2=f2+0.1; //0.08
        sudutNgurang=f2;
        tempx=x;//-sin(sudutNgurang*3.14/180)*2;
        tempy=y-y*sin(sudutNgurang*3.14/180);//2.4
        tempz=z;

        temp_sudutX=sudutX-sudutX*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        temp_sudutY=sudutY-3*sin(sudutNgurang*3.14/180);//+2*sin(sudutNgurang*3.14/180);
        temp_sudutZ=sudutZ;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));

        tempx2=x2;//+sin(sudutNgurang*3.14/180)*2;
        tempy2=y2-y2*sin(sudutNgurang*3.14/180);//2.4
        tempz2=z2;//0.2;//.2;

        temp_sudutX2=sudutX2*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2)); +4*sin
        temp_sudutY2=sudutY2-3*sin(sudutNgurang*3.14/180);//+2*sin(sudutNgurang*3.14/180);
        temp_sudutZ2=sudutZ2;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2
        speed=50;//cos(sudutNgurang*3.14/360);

//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        inverseAndre(temp_sudutX,temp_sudutY,temp_sudutZ,tempx,tempy,tempz,temp_sudutX2,temp_sudutY2,temp_sudutZ2,tempx2,tempy2,tempz2,speed,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
        //usleep(1000);
//       cout<<"Masukkan sudut X: ";
                }
        f2=0;

         //cout<<"Masukkan sudut X: ";
        usleep(1000000); //usleep(500000);
	//sleep(1);//2

							        // MIRING KIRI//
        while(f2<=90)

        {f2=f2+0.08;//0.08
        sudutNgurang=f2;
        x=tempx;//+sin(sudutNgurang*3.14/180)*2;
        y=tempy+6*sin(sudutNgurang*3.14/180);			//+ ke kiri		//+6
        z=tempz;//0.5;//cos(sudutNgurang*3.14/180);

        sudutX=temp_sudutX;//-5*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));  		//-1 sebelum 
 	sudutY=temp_sudutY+2*sin(sudutNgurang*3.14/180);
        sudutZ=temp_sudutZ;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=tempx2;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=tempy2+2*sin(sudutNgurang*3.14/180);			//+3
        z2=tempz2;//0.2;//cos(sudutNgurang*3.14/180);

        sudutX2=temp_sudutX2-0*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2)); 	//-10
        sudutY2=temp_sudutY2+2*sin(sudutNgurang*3.14/180);
        sudutZ2=temp_sudutZ2;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=50;//cos(sudutNgurang*3.14/360);
//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
                        }
        f2=0;
        usleep(100000); //60000
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
        f2=f2+0.05; //0.08
        sudutNgurang=f2;
        tempx=x+2*sin(sudutNgurang*3.14/180);
        tempy=y+0*sin(sudutNgurang*3.14/180);//1+1*sin(sudutNgurang*3.14/180);//- pidy;		//2
        tempz=z+1*sin(sudutNgurang*3.14/360); //sebelum 2		//2

        temp_sudutX=sudutX-0*sin(sudutNgurang*3.14/180);//- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/ 	//15 sebelum b
        temp_sudutY=sudutY;//-2*sin(sudutNgurang*3.14/180);//+0*sin(sudutNgurang*3.14/180);//+sin(sudutNgurang*3.14/180);
        temp_sudutZ=sudutZ;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));

        tempx2=x2+0*sin(sudutNgurang*3.14/180);
        tempy2=y2+0*sin(sudutNgurang*3.14/180);				//2
        tempz2=z2-0.5*sin(sudutNgurang*3.14/360); 			//-1
 	//z2=-1*cos(sudutNgurang*3.14/180);

        temp_sudutX2=sudutX2;//+12*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        temp_sudutY2=sudutY2;//-2*sin(sudutNgurang*3.14/180);//+2*sin(2*sudutNgurang*3.14/360);
        temp_sudutZ2=sudutZ2;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=50;//+50*cos(sudutNgurang*3.14/360);
	/*cout<<temp_sudutX<<endl<<temp_sudutY<<endl<<temp_sudutZ<<endl<<tempx
	<<endl<<tempy<<endl<<tempz<<endl;*/
 	inverseAndre(temp_sudutX,temp_sudutY,temp_sudutZ,tempx,tempy,tempz,temp_sudutX2,temp_sudutY2,temp_sudutZ2,tempx2,tempy2,tempz2,speed,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);
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
        sudutY=temp_sudutY+3*sin(sudutNgurang*3.14/180);//-1*sin(sudutNgurang*3.14/180);
        sudutZ=temp_sudutZ;//3*(pow(sin(sudutNgurang*3.14/180),2));

        x2=tempx2;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=tempy2-tempy2*sin(sudutNgurang*3.14/180);//2.4
        z2=tempz2;//+1*sin(sudutNgurang*3.14/180);//-0.5;//-0.5;//cos(sudutNgurang*3.14/180);

        sudutX2=temp_sudutX2*sin(sudutNgurang*3.14/180);
        sudutY2=temp_sudutY2+3*sin(sudutNgurang*3.14/180);//-1*sin(sudutNgurang*3.14/180);
        sudutZ2=temp_sudutZ2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);

//      inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',480,520,270,750,250,750,data_image.x_head,data_image.y_head);

                }
        f2=0;
	/*cout<<x<<endl<<y<<endl<<z<<endl<<sudutX<<endl<<sudutY<<endl<<sudutZ<<endl;
	cout<<x2<<endl<<y2<<endl<<z2<<endl<<sudutX2<<endl<<sudutY2<<endl<<sudutZ2<<endl;*/
                //usleep(500000);
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
void cari_garis(){
  cout<<data_image.x_head<<endl;
  if(garis.tengah<3){
    putarkanan();
    //sleep(1);
    //berdiri();
  }
}
void decide_walk(){
 // std::lock_guard<std::mutex> lockGuard(mutex_garis);
  cout<<"tengah: "<<garis.tengah<<endl
  <<"kanan: "<<garis.kanan<<endl
  <<"kiri: "<<garis.kiri<<endl;
  if(garis.tengah<garis.kanan){
    jalan();
  }
  else{
    cari_garis();
    sleep(1);
  }
}
int main(){
	//int fd;
	dxl_initialize(0,1);
//        dxl_write_word(19,32,800);
//        dxl_write_word(20,32,800);
	data_image.y_head=310;
	data_image.x_head=548;
	data_image.condition=1;	
	//berdiri();
	putarkanan();
	sleep(1);
	//sleep(10);
//	char * myfifo="/tmp/myfifo";
//	mkfifo(myfifo,0666);
//	init_cam();
//	read_cam();
	while(1){
		putarkanan();
//		read_cam();
//		cout<<garis.kanan<<" "<<garis.tengah<<" "<<garis.kiri<<endl;
//		decide_walk();
//		std::thread th_walk(decide_walk);
//		std::thread th_cam(read_cam);
//		th_cam.join();
//		th_walk.join();
//		fd=open(myfifo,O_RDONLY);
//		read(fd,&data_image,sizeof(data_image));
//		close(fd);
//		std::cout<<"cmd_img "<<data_image.condition<<endl<<endl;
		/*switch(data_image.condition){
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
		}*/
//		cout<<garis.tengah<<endl;
//		usleep(1000);
		sleep(1);
	}
	return 0;
}

