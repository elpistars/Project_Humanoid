#ifndef AYA_C
#define AYA_C

#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

double nilaisensor,epx,edx,epy,edy,erry,outpx,outpy,torka,torki;
double kpx2 = 5.025;
double kdx = 0.4;
double kpx = 12.025;
double kdx2 = 1.4;
			double kpy = 20.014;
			double kdy = 6.04;//.125;//;.0068;
double outpx2=0;
double outdx = 0;
double outdx2 = 0;
double outdy = 0;
double epx2=0;
double edx2=0;
double edysblm = 0;
double edxsblm = 0;
double edx2sblm = 0;
double pidx = 0;
double pidx2 = 0;
double pidy= 0;
double setpointx = 184.5;
double setpointy = 1;
double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;


void sensen(){
	
//	string status;
	enableIMU();
//	status=bacaACC(Pacc_raw);
//	cout<<status<<endl;
	//bacaMAG(Pmag_raw, 1);
	nilaisensor =  bacaACC2(Pacc_raw);//AccXangle;
	//sleep(1);
	
}


void verse(){
	
	//double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
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
        sudutY=7;
        sudutZ=0;
        x2=-14;
        y2=0;
        z2=0;
        sudutX2=0;
        sudutY2=7;
        sudutZ2=0;

	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,100,50,'2');
	cout<<"Masukkan Speed: ";
	//cin>>speed;
	sleep(7);
		//kirimPacketInverse(x,y);

	//for(int i=1;i<=10;i++){
	// MIRING //
	
     	

/*         for(int i=0;i<=38;i++){
        //cout<<"Masukkan X: ";
         sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=1.1*sin(sudutNgurang*3.14/180);//2.4
        z=1;
        sudutX=-3*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        sudutY=7;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*2;
        y2=1.1*sin(sudutNgurang*3.14/180);//2.4
        z2=0;
        sudutX2=3*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
        sudutY2=7;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
          speed=100;//cos(sudutNgurang*3.14/360);
//===============
         sensen();
     //   pidtangan();
        //=================
        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
        usleep(2000);
	}
	
	usleep(100);	
	//  ANKAT KAKI DAN MENDARAT //

	 for(int i=0;i<=38;i++){

	
	 sudutNgurang=2.4*i;
	x=-14+4*sin(sudutNgurang*3.14/180)*1.4;
        y=1.1+0*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
        z=0;//0.5-0.5*cos(sudutNgurang*3.14/180);
	sudutX=-5.5-6*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));	
	sudutY=7;//+4*cos(sudutNgurang*3.14/180);
        sudutZ=0;//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-2*sin(sudutNgurang*3.14/180);
      	y2=1.1+4*sin(sudutNgurang*3.14/180); //+ pidy;//sin(sudutNgurang*3.14/180);//2.4
      	z2=0;//-0.5+0.5*cos(sudutNgurang*3.14/180);
        sudutX2=5.5-13*sin(sudutNgurang*3.14/180);// - pidx;(pow(sin(sudutNgurang*3.14/180),2));
        sudutY2=7;//+4*sin(sudutNgurang*3.14/180);
    sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	speed=100;//cos(sudutNgurang*3.14/360);
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	//baca sensor
	sensen();
	
//		if(i>46){
			//PID x
			// P x
			epx = setpointx - nilaisensor;
			outpx = kpx*epx;
			//D x
			edx = epx - edxsblm;
			outdx = kdx*edx;
			edxsblm = epx;

			pidx = outpx + outdx;
			
			//PID y
			// P y
			erry = cos(epx) + sin(epx);
			epy = setpointy - erry;
			outpy = kpy*epy;
			// D y
			edy = epy - edysblm;
			outdy = kdy*edy;
			edysblm = epy;
			
			pidy = outpy + outdy;
			}
	usleep(1000);
	}
	//sleep(30000);

*/
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

while(1){
	
	sensen();
	 epx = setpointx - nilaisensor;
                        
		 outpx = kpx*epx;
                        //D x
                        edx = epx - edxsblm;
                        outdx = kdx*edx;
                        edxsblm = epx;

                        pidx = outpx + outdx;
	


		outpx2 = kpx2*epx2;
                        //D x
                        edx2 = epx2 - edx2sblm;
                        outdx2 = kdx2*edx2;
                        edx2sblm = epx2;

                        pidx2 = outpx2 + outdx2;

                        //PID y
                       //if (epx < 0){
			//erry = -cos(epx*0.017);
			//} else
			//if (epx > 0){
			erry = cos(epx*0.017);
			
			 // P y
                      //  erry = cos(epx);// + sin(epx);
                        epy = setpointy - erry;
                        outpy = kpy*epy;
                        // D y
                        edy = epy - edysblm;
                        outdy = kdy*edy;
                        edysblm = epy;

                        pidy = outpy + outdy;
		torka=200+pidy*0.01/0.0014667;
		if (torka < 0){
                        torka = -torka;
                        } else
                        if (torka > 0){
                        torka = torka;
			}else 
			if (torka>1023){
			torka=1023;
                    	}
		if (epx < 0){
                        epy = -epy;//cos(epx*0.017);
                        } else
                        if (epx > 0){
			epy = epy;//cos(epx*0.017);
			}
		y2=-0.05*epy;
		y=-0.05*epy;
//sudutX2=-8.5-pidx2;
		//sudutX=-11.5+pidx;	
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,torka,torka,'2');
//	 cout<<"pidx"<<pidx<<endl
	 cout<<"pidy "<<torka<<endl;
	//cout<<"y" <<y<<endl;
//	usleep(1000);	
//sleep(1);
}
//return 0;
}

#endif
