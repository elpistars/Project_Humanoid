#ifndef PUTARKANAN_CPP
#define PUTARKANAN_CPP

#include "/home/pi/Multithreading/library/allLibtune.h"
//#include "/home/pi/Multithreading/Geraklangkah.cpp"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

double f2;
double a=0.2;

void putarkanan(){
	
	double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;	
	int speed, com7, com8;
	// MIRING //
	f2=0;
	while(f2<=90.0)
   {
	f2=f2+a;
	sudutNgurang=f2;
        x=-10.4;//-2*sin(sudutNgurang*3.14/180)*2;
        y=1*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=-5;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
	x2=-10.4;//+sin(sudutNgurang*3.14/180)*2;
      	y2=1*sin(sudutNgurang*3.14/180);//2.4
      	z2=0;
        sudutX2=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=-5;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
	speed=50;//cos(sudutNgurang*3.14/360);
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500);

   }
	f2=0;
	
// ------------------  ANKAT KAKI DAN MENDARAT //
	while(f2<=180)
	{
		f2=f2+a;
		sudutNgurang=f2;
		x=-10.4+2*sin(sudutNgurang*3.14/180);
        	y=-1-0*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
	        z=0;//0+0.8-0.8*cos(sudutNgurang*3.14/180);
		sudutX=2+20*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	
		sudutY=-5;//+6*sin(sudutNgurang*3.14/360);
	        sudutZ=512+30*sin(sudutNgurang*3.14/360);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
	        x2=-11.4-0*sin(sudutNgurang*3.14/180);
	      	y2=-1-1*sin(sudutNgurang*3.14/180);// + pidy;//sin(sudutNgurang*3.14/180);//2.4
	      	z2=0;//-0.8+0.8*cos(sudutNgurang*3.14/180);
	        sudutX2=-2+16*sin(sudutNgurang*3.14/180);// - pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
	        sudutY2=-5;//-4*sin(sudutNgurang*3.14/360);
		sudutZ2=512;//+10*cos(sudutNgurang*3.14/360);//-3*(pow(sin(sudutNgurang*3.14/180),2));
		speed=50;//+50*cos(sudutNgurang*3.14/360);	
		inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,speed,sudutZ,sudutZ2,'2',500,500,250,780,500,500);

	}

	f2=0;
	
// --------------------------- BALIKIN MIRING //
	while(f2<=90)
	{
		f2=f2+a;
		sudutNgurang=f2;
	        x=-11.4;//-sin(sudutNgurang*3.14/180)*2;
	        y=-1+1*sin(sudutNgurang*3.14/180);//2.4
	        z=0;//1.6;
	        sudutX=2-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
	        sudutY=-5;//-6*sin(sudutNgurang*3.14/180);
	        sudutZ=542;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
	        x2=-11.4;//+sin(sudutNgurang*3.14/180)*2;
	        y2=-1+1*sin(sudutNgurang*3.14/180);//2.4
	        z2=0;//-1.6;//0.2;//.2;
	        sudutX2=-2+2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
	        sudutY2=-5;//+4*sin(sudutNgurang*3.14/180);
	        sudutZ2=512;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2 
		speed=100;//cos(sudutNgurang*3.14/360);
		inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,speed,sudutZ,sudutZ2,'2',500,500,250,780,500,500);

	}
	f2=0;
	
	sleep(50000);

// ----------------------------- MIRING //	
/*		while(f2<=90)
		{
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180)*2;
			y=1*sin(sudutNgurang*3.14/180);//2.4
			z=0;//0.5;//cos(sudutNgurang*3.14/180);
			sudutX=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
/*			sudutY=0;//+1*sin(sudutNgurang*3.14/180);
			sudutZ=542;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*2.4;
			y2=1*sin(sudutNgurang*3.14/180);//2.4
			z2=0;//0.2;//cos(sudutNgurang*3.14/180);
			sudutX2=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
/*			sudutY2=0;//+1*sin(sudutNgurang*3.14/180);
			sudutZ2=512;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//cos(sudutNgurang*3.14/360);
			inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,speed,sudutZ,sudutZ2,'2');
		}
	
// ------------------------ ANGKAT KAKI DAN MENDARAT/
		f2=0;
		
		while(f2<=180){
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4+0*sin(sudutNgurang*3.14/180);
			y=1+3*sin(sudutNgurang*3.14/180);//- pidy;//2.4
			z=0;//1.2*cos(sudutNgurang*3.14/180);
			sudutX=-2+8*sin(sudutNgurang*3.14/180);//- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
/*			sudutY=0-4*sin(sudutNgurang*3.14/360);//+sin(sudutNgurang*3.14/180);
			//speed=200*cos(sudutNgurang*3.14*0.5/180);
			sudutZ=542-30*sin(sudutNgurang*3.14/360);//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4+2*sin(sudutNgurang*3.14/180);
			y2=1+0*sin(sudutNgurang*3.14/180);//2.4
			z2=0.4-0.4*cos(sudutNgurang*3.14/180);
			sudutX2=2+16*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
/*			sudutY2=0;//+2*sin(sudutNgurang*3.14/360);
			sudutZ2=512;//+10*sin(sudutNgurang*3.14/360);//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=50;//+50*sin(sudutNgurang*3.14/180);
			inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,speed,sudutZ,sudutZ2,'2');
		}
		f2=0;
		// BALIKIN MIRING//
		
		while(f2<=90){
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180)*1.4;
			y=1-1*sin(sudutNgurang*3.14/180);//2.4
			z=0;//cos(sudutNgurang*3.14/180);
			sudutX=-2+2*sin(sudutNgurang*3.14/180);
			sudutY=-4+4*sin(sudutNgurang*3.14/180);
			sudutZ=512;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*1.4;
			y2=1-1*sin(sudutNgurang*3.14/180);//2.4
			z2=0.8;//cos(sudutNgurang*3.14/180);
			sudutX2=2-2*sin(sudutNgurang*3.14/180);
			sudutY2=0;//+2*sin(sudutNgurang*3.14/180);
			sudutZ2=512;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
			inverseAndre(sudutX,sudutY,0,x,y,z,sudutX2,sudutY2,0,x2,y2,z2,speed,sudutZ,sudutZ2,'2');}
*/		
		
		f2=0;
		//usleep(500000);
		
//		int id[] = {20};
//		int data1[][18] = {512};
//		int delay1[] = {1000};
//		int speed1[] = {100};
//		kirimPacketGerak(1, 1, 1, id, data1, 1, delay1, speed1);
		
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

        putarkanan();
usleep(3000);
}
return 0;
        }



#endif
