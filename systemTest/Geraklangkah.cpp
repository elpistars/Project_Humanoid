#ifndef GERAKLANGKAH_CPP
#define GERAKLANGKAH_CPP

#include "/home/pi/Multithreading/library/allLib.h"

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
double x_gerak,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
int speed, com7, com8,comnentu,delayberdiri;

void berdiri(){
	
	dxl_write_word(10,26,1);
    	dxl_write_word(10,27,1);
  	dxl_write_word(9,26,1);
  	dxl_write_word(9,27,1);


    dxl_write_word(10,28,4);
    dxl_write_word(10,29,4);
	dxl_write_word(9,28,4);
	dxl_write_word(9,29,4);


	dxl_write_word(1,30,492); // maju tambah  ;;  mundur kurang
    dxl_write_word(2,30,532);
    dxl_write_word(3,30,300);
    dxl_write_word(4,30,724);
    dxl_write_word(5,30,350); //200
    dxl_write_word(6,30,650); //800

    // kaki kanan
	x_gerak=-11.4;// tinggi kaki
    y=0;// miring kaki  ;; ankle
    z=0;// langkah kaki ;; ++ maju
    sudutX=0;//miring badan  ;;   makin nambah makin ke kanan
    sudutY=0;//kebungkukan badan   ;; bungkuk nambah
    sudutZ=0;//putaran badan
        // kaki kiri
    x2=-11.4;
    y2=0;//3;
    z2=0;
    sudutX2=0;//-6;
    sudutY2=0;
    sudutZ2=0;
	inverseAndre(sudutX,sudutY,sudutZ,x_gerak,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,512,512,'2',500,500,250,780,500,500);

	usleep(delayberdiri);
		}
		
void langkah(){		
	//--------- LANGKAH 1---------------
	f2=0;
    while(f2<=90.0)
	
   {
	f2=f2+a;
	sudutNgurang=f2;
        x=-11.4;//-2*sin(sudutNgurang*3.14/180)*2;
        y=-0.5*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=0;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
	x2=-11.4;//+sin(sudutNgurang*3.14/180)*2;
      	y2=-0.5*sin(sudutNgurang*3.14/180);//2.4
      	z2=0;
        sudutX2=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=0;//-sin(sudutNgurang*3.14/180);
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
		x=-11.4+2*sin(sudutNgurang*3.14/180);
        	y=-0.5-0*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
	        z=0+0.4-0.4*cos(sudutNgurang*3.14/180);
		sudutX=2+8*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	
		sudutY=0+3*sin(sudutNgurang*3.14/360);
	        sudutZ=0;//+20*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
	        x2=-11.4-0*sin(sudutNgurang*3.14/180);
	      	y2=-0.5-2.5*sin(sudutNgurang*3.14/180);// + pidy;//sin(sudutNgurang*3.14/180);//2.4
	      	z2=-0.4+0.4*cos(sudutNgurang*3.14/180);
	        sudutX2=-2+12*sin(sudutNgurang*3.14/180);// - pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
	        sudutY2=0-7*sin(sudutNgurang*3.14/360);
		sudutZ2=0;//+20*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
		speed=300;//+50*cos(sudutNgurang*3.14/360);
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500);
	}

	f2=0;
	
// --------------------------- BALIKIN MIRING //
	while(f2<=90)
	{
		f2=f2+a;
		sudutNgurang=f2;
	        x=-11.4;//-sin(sudutNgurang*3.14/180)*2;
	        y=-0.5+0.5*sin(sudutNgurang*3.14/180);//2.4
	        z=0.8;
	        sudutX=2-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
	        sudutY=3-3*sin(sudutNgurang*3.14/180);
	        sudutZ=0;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
	        x2=-11.4;//+sin(sudutNgurang*3.14/180)*2;
	        y2=-0.5+0.5*sin(sudutNgurang*3.14/180);//2.4
	        z2=-0.8;//0.2;//.2;
	        sudutX2=-2+2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
	        sudutY2=-7+7*sin(sudutNgurang*3.14/180);
	        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2 
		speed=200;//cos(sudutNgurang*3.14/360);
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
	}
	f2=0;
	
	//-----------------LANGKAH 2---------------	
	
//	usleep(5000);
// ----------------------------- MIRING //	
		while(f2<=90)
		{
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180)*2;
			y=0.5*sin(sudutNgurang*3.14/180);//2.4
			z=0.8;//0.5;//cos(sudutNgurang*3.14/180);
			sudutX=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY=0;//+1*sin(sudutNgurang*3.14/180);
			sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*2.4;
			y2=0.5*sin(sudutNgurang*3.14/180);//2.4
			z2=-0.8;//0.2;//cos(sudutNgurang*3.14/180);
			sudutX2=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=0;//+1*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=50;//cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');      
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500);
		}
	
// ------------------------ ANGKAT KAKI DAN MENDARAT/
		f2=0;
			
		while(f2<=180){
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4+0*sin(sudutNgurang*3.14/180);
			y=0.5+3*sin(sudutNgurang*3.14/180);//- pidy;//2.4
			z=0.8*cos(sudutNgurang*3.14/180);
			sudutX=-2+12*sin(sudutNgurang*3.14/180);//- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY=0-2*sin(sudutNgurang*3.14/360);//+sin(sudutNgurang*3.14/180);
			//speed=200*cos(sudutNgurang*3.14*0.5/180);
			sudutZ=0;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4+2*sin(sudutNgurang*3.14/180);
			y2=1+0*sin(sudutNgurang*3.14/180);//2.4
			z2=-0.8*cos(sudutNgurang*3.14/180);
			sudutX2=2+8*sin(sudutNgurang*3.14/180);//24 sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=0;//+2*sin(sudutNgurang*3.14/360);
			sudutZ2=0;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=200;//+50*sin(sudutNgurang*3.14/180);
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
		}
		f2=0;
		// BALIKIN MIRING//
		
		while(f2<=90){
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180)*1.4;
			y=0.5-0.5*sin(sudutNgurang*3.14/180);//2.4
			z=-0.8;//cos(sudutNgurang*3.14/180);
			sudutX=-2+2*sin(sudutNgurang*3.14/180);
			sudutY=-2+2*sin(sudutNgurang*3.14/180);
			sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*1.4;
			y2=0.5-0.5*sin(sudutNgurang*3.14/180);//2.4
			z2=0.8;//cos(sudutNgurang*3.14/180);
			sudutX2=2-2*sin(sudutNgurang*3.14/180);
			sudutY2=0;//+2*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
		}
		
		f2=0;
		
//		usleep(5000);
		//-----------------KEMBALI TEGAK-------------
	// ---------------------------- MIRING //
		while(f2<=90){
			f2=f2+a; 
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180)*2;
			y=-0.5*sin(sudutNgurang*3.14/180);//2.4
			z=-0.8;//cos(sudutNgurang*3.14/180);
			sudutX=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY=0;//+2*sin(sudutNgurang*3.14/180);
			sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*2.4;
			y2=-0.5*sin(sudutNgurang*3.14/180);//2.4
			z2=0.8;//-0.2;//cos(sudutNgurang*3.14/180);
			sudutX2=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=0;//+2*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=50;//cos(sudutNgurang*3.14/360);
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
		}		
	// ------------------------ ANGKAT KAKI DAN MENDARAT//
		f2=0;
		
		while(f2<=180){
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4+3*sin(sudutNgurang*3.14/180);
			y=-0.5-0*sin(sudutNgurang*3.14/180);//2.4
			z=-0.8+0.8*sin(sudutNgurang*3.14/360);
			sudutX=2+10*sin(sudutNgurang*3.14/180);///12*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY=0+3*sin(sudutNgurang*3.14/360);
			sudutZ=0;//cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//+sin(sudutNgurang*3.14/180)*1.5;
			y2=-0.5-2.5*sin(sudutNgurang*3.14/180);//+pidy;//2.4
			z2=0.8-0.8*sin(sudutNgurang*3.14/360);
			sudutX2=-2+15*sin(sudutNgurang*3.14/180);//16- pidx;//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=0+3*sin(sudutNgurang*3.14/360);//-sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=200;//1+100*cos(sudutNgurang*3.14/360);
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500);
		}	
		f2=0;
	
	// ---------------------- BALIKIN MIRING//
		while(f2<=90){
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180)*1.4;
			y=-0.5+0.5*sin(sudutNgurang*3.14/180);//2.4
			z=0;//cos(sudutNgurang*3.14/180);
			sudutX=5-5*sin(sudutNgurang*3.14/180);
			sudutY=3-3*sin(sudutNgurang*3.14/180);
			sudutZ=0;//-2;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*1.4;
			y2=-0.5+0.5*sin(sudutNgurang*3.14/180);//2.4
			z2=0;//0.2;//cos(sudutNgurang*3.14/180);
			sudutX2=-2+2*sin(sudutNgurang*3.14/180);
			sudutY2=3-3*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-3(pow(sin(sudutNgurang*3.14/180),2));
			speed=50;//cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2',500,500,250,780,500,500);
		}
		
		usleep(1000000);
	
	}

#endif
