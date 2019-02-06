#ifndef INVERSETEST_C
#define INVERSETEST_C

#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

int main(){

if(dxl_initialize(0,1)==0){
        cout<<"Error connecting"<<endl;
        return 0;
    }
    
else{
		cout<<"connect success!"<<endl;
	}

while(1){
	double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
	int speed;

/*        cout<<"Masukkan sudut X: ";
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
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=0;
        z=0;
        sudutX=0;
        sudutY=5;
        sudutZ=0;
        x2=-14;
        y2=0;
        z2=0;
        sudutX2=0;
        sudutY2=5;
        sudutZ2=0;

	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
	cout<<"Masukkan Speed: ";
	//cin>>speed;
	sleep(5);
		//kirimPacketInverse(x,y);

	//for(int i=1;i<=10;i++){
	// MIRING //
	
     	
	 for(int i=0;i<=38;i++){
       	//cout<<"Masukkan X: ";
	 sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=1.1*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=-5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=5;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
	x2=-14;//+sin(sudutNgurang*3.14/180)*2;
      	y2=1.1*sin(sudutNgurang*3.14/180);//2.4
      	z2=0;
        sudutX2=5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=5;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
	  speed=300*cos(sudutNgurang*3.14/360);

	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
//	sleep(1);
	}
	
	usleep(100);	
	//  ANKAT KAKI DAN MENDARAT //

	 for(int i=0;i<=38;i++){
	
	
	
	 sudutNgurang=2.4*i;
	x=-14+2*sin(sudutNgurang*3.14/180)*1.4;
        y=1.1+2*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
        z=1-1*cos(sudutNgurang*3.14/180);
	sudutX=-5.5-10*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	
	sudutY=5+4*cos(sudutNgurang*3.14/180);
        sudutZ=0;//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+7*sin(sudutNgurang*3.14/180);
      	y2=1.1+5*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);//2.4
      	z2=-1+1*cos(sudutNgurang*3.14/180);
        sudutX2=5.5-18*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=5+4*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	speed=300*cos(sudutNgurang*3.14/360);
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	//baca sensor

	}
	sleep(100);
	// BALIKIN MIRING //

        for(int i=0;i<=38;i++){
	 sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=1.1-1.1*sin(sudutNgurang*3.14/180);//2.4
        z=2;
        sudutX=-5.5+5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=5+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*2;
        y2=1.1-1.1*sin(sudutNgurang*3.14/180);//2.4
        z2=-2;//0.2;//.2;
        sudutX2=5.5-5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=5+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2 
	speed=300*cos(sudutNgurang*3.14/360);
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');     
 }
	sleep(1000);
	// MIRING //	
for(int j=0;j<=76;j++){


	for(int i=0;i<=38;i++){
	 sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*2;
        y=-2.5*sin(sudutNgurang*3.14/180);//2.4
        z=2;//cos(sudutNgurang*3.14/180);
        sudutX=-5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7+3*sin(sudutNgurang*3.14/180);
        sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=-2.5*sin(sudutNgurang*3.14/180);//2.4
        z2=-2;//0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=-5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7+3*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
         speed=300*cos(sudutNgurang*3.14/360);
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');      
		}
        usleep(100);
	//ANGKAT KAKI DAN MENDARAT//

	for(int i=0;i<=76;i++){
 	 sudutNgurang=2.4*i;
        x=-14;//sin(sudutNgurang*3.14/180)*3
	y=-2.5-2*sin(sudutNgurang*3.14/180);//2.4
        z=2*cos(sudutNgurang*3.14/180);
        sudutX=-5-9*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=10+3*sin(sudutNgurang*3.14/180);//+sin(sudutNgurang*3.14/180);
	// speed=200*cos(sudutNgurang*3.14*0.5/180);
        sudutZ=0;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14+3*sin(sudutNgurang*3.14/180)*1.5;
        y2=-2.5-1*sin(sudutNgurang*3.14/180);//2.4
        z2=-2*cos(sudutNgurang*3.14/180);
        sudutX2=-5-10*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10+3*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	 speed=300*cos(sudutNgurang*3.14/360);

	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	usleep(100);
			}

	usleep(100);
	// BALIKIN MIRING//
	
	for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*1.4;
        y=-2.5+2.5*sin(sudutNgurang*3.14/180);//2.4
        z=-2;//cos(sudutNgurang*3.14/180);
	sudutX=-5+5*sin(sudutNgurang*3.14/180);
        sudutY=10;//-1*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=-2.5+2.5*sin(sudutNgurang*3.14/180);//2.4
        z2=2;//-0.2;//cos(sudutNgurang*3.14/180);
	sudutX2=-5+5*sin(sudutNgurang*3.14/180);
        sudutY2=10;//-1*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
         speed=300*cos(sudutNgurang*3.14/360);
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');

		}
		usleep(1000);
   // MIRING //


	for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*2;
        y=3*sin(sudutNgurang*3.14/180);//2.4
        z=-2;//cos(sudutNgurang*3.14/180);
        sudutX=-5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=10+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=3*sin(sudutNgurang*3.14/180);//2.4
        z2=2;//-0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=-5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=300*cos(sudutNgurang*3.14/360);
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
			}

   usleep(100);
        //ANGKAT KAKI DAN MENDARAT//

        for(int i=0;i<=76;i++){
         sudutNgurang=2.4*i;
        x=-14+3*sin(sudutNgurang*3.14/180)*1.5;
        y=3+1*sin(sudutNgurang*3.14/180);//2.4
        z=-2*cos(sudutNgurang*3.14/180);
        sudutX=-5-10*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=12+3*sin(sudutNgurang*3.14/180);
        sudutZ=0;//cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*1.5;
        y2=3+2*sin(sudutNgurang*3.14/180);//2.4
        z2=2*cos(sudutNgurang*3.14/180);
        sudutX2=-5-9*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=12+3*sin(sudutNgurang*3.14/180);//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=300*cos(sudutNgurang*3.14/360);
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');

			}

   usleep(100);
        // BALIKIN MIRING//

        for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*1.4;
        y=3-3*sin(sudutNgurang*3.14/180);//2.4
        z=2;//cos(sudutNgurang*3.14/180);
        sudutX=-5+5*sin(sudutNgurang*3.14/180);
        sudutY=12;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//-2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=3-3*sin(sudutNgurang*3.14/180);//2.4
        z2=-2;//0.2;//cos(sudutNgurang*3.14/180);
        sudutX=-5+5*sin(sudutNgurang*3.14/180);
        sudutY2=12;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=300*cos(sudutNgurang*3.14/360);
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');

		}
	usleep(1000);
	}
	sleep(30000);
}
}

#endif
