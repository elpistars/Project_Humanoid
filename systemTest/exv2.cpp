#ifndef AYANYAN_C 
#define AYANYAN_C

#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

float nilaisensor;
double epka,epki,edka,edki,epx,edx,epy,edy,erry,outpx,outpy,outpka,outpki,outdka,outdki,outdx,outdy;
double kpx = 0;//.025;
double kdx = 0;//.4;
double kpy = 0;//.014;
double kdy = 0;//.0068;
double kpka = 0;//1;
double kdka = 0;
double kpki = 0;
double kdki = 0;
double edysblm = 0;
double edkasblm = 0;
double edkisblm = 0;
double edxsblm = 0;
double pidx = 0;
double pidy= 0;
double setpointx = 0;//184.5;
double setpointy = 0;//1.1;
double pidka = 0;
double pidki = 0;
double tkan,tki;

void pidtangan(){
double edkasblm = 0;
double edkisblm = 0;

	
		//PID tangan ka
			// P ka
			epka = setpointx - nilaisensor;
			outpka = kpka*epka;
			//D ka
			edka = epka - edkasblm;
			outdka = kdka*edka;
			edkasblm = epka;

			pidka = outpka + outdka;
			tkan = 300 ;//+ pidka;
			//PID tangan ki
			// P ki
			epki = setpointx - nilaisensor;
			outpki = kpki*epki;
			//D ki
			edki = epki - edkisblm;
			outdki = kdki*edki;
			edkisblm = epki;

			pidki = outpki + outdki;
			tki = 744;// + pidki;
}

void sensen(){

	//string status;
	//readACC(Pacc_raw);
	//float AccXangle = (float) (atan2(*(Pacc_raw+1),*(Pacc_raw+2))+M_PI)*RAD_TO_DEG;
	enableIMU();
	//status=bacaACC2(Pacc_raw);
	//cout<<status<<endl;
	//bacaMAG(Pmag_raw, 1);
	nilaisensor = bacaACC2(Pacc_raw);
	//sleep(1);
	
}


void verse(){
	
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
       //while(1){
	 x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=0;
        z=1;
        sudutX=0;
        sudutY=6;
        sudutZ=0;
        x2=-14;
        y2=0;
        z2=0;
        sudutX2=0;
        sudutY2=6;
        sudutZ2=0;
	//============
	sensen();
	pidtangan();
//	tkan = 300;
//	t8ki = 744;
	//===============
	 inverseAndre(tkan,tki,sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
	cout<<"Masukkan Speed: ";
	//cin>>speed;
	/*cout<<"tkan : "<<tkan<<endl;
	cout<<"tki : "<<tki<<endl;
 cout<<"pidka : "<<pidka<<endl;
        cout<<"pidki : "<<pidki<<endl;
 cout<<"nilaisensor : "<<nilaisensor<<endl;
        */
	sleep(5);
		//kirimPacketInverse(x,y);
//}
	//for(int i=1;i<=10;i++){
	// MIRING //
	
     	
	 for(int i=0;i<=38;i++){
       	//cout<<"Masukkan X: ";
	 sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=1.1*sin(sudutNgurang*3.14/180);//2.4
        z=1;
        sudutX=-3*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
	x2=-14;//+sin(sudutNgurang*3.14/180)*2;
      	y2=1.1*sin(sudutNgurang*3.14/180);//2.4
      	z2=0;
        sudutX2=3*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
	  speed=100;//cos(sudutNgurang*3.14/360);
//===============
	  sensen();
	pidtangan();
	//=================
	inverseAndre(tkan,tki,sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	usleep(2000);
	}
	
	usleep(1000);	
	//  ANKAT KAKI DAN MENDARAT //

	 for(int i=0;i<=76;i++){

	
	 sudutNgurang=2.4*i;
	x=-14+4*sin(sudutNgurang*3.14/180)*1.4;
        y=1.1+6*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
        z=1+1-1*cos(sudutNgurang*3.14/180);
	sudutX=-3-15*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	
	sudutY=7+4*cos(sudutNgurang*3.14/180);
        sudutZ=0;//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-2*sin(sudutNgurang*3.14/180);
      	y2=1.1+10*sin(sudutNgurang*3.14/180) + pidy;//sin(sudutNgurang*3.14/180);//2.4
      	z2=-1+1*cos(sudutNgurang*3.14/180);
        sudutX2=3-16*sin(sudutNgurang*3.14/180) - pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7+4*sin(sudutNgurang*3.14/180);
    sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	speed=100;//cos(sudutNgurang*3.14/360);
	//============
	sensen();
	pidtangan();
	//============
	inverseAndre(tkan,tki,sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	//baca sensor
	
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
//			}
	usleep(1000);
	}
	//usleep(70);
	// BALIKIN MIRING //

        for(int i=0;i<=38;i++){
	 sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=1.1-1.1*sin(sudutNgurang*3.14/180);//2.4
        z=3;
        sudutX=-3+3*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*2;
        y2=1.1-1.1*sin(sudutNgurang*3.14/180);//2.4
        z2=-2;//0.2;//.2;
        sudutX2=3-3*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2 
	speed=100;//cos(sudutNgurang*3.14/360);
	//=============
	sensen();
	pidtangan();
	//====================
	inverseAndre(tkan,tki,sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');     
usleep(2000);

}
	usleep(1000);
	// MIRING //	
for(int j=0;j<=76;j++){


	for(int i=0;i<=38;i++){
	 sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*2;
        y=-0.5*sin(sudutNgurang*3.14/180);//2.4
        z=3;//cos(sudutNgurang*3.14/180);
        sudutX=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7-2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=-0.5*sin(sudutNgurang*3.14/180);//2.4
        z2=-2;//0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7-2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
         speed=100;//cos(sudutNgurang*3.14/360);
		 //================
		 sensen();
	pidtangan();
	//================
	 inverseAndre(tkan,tki,sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');      
usleep(2000);
		}
        //usleep(100);
	//ANGKAT KAKI DAN MENDARAT/
	
	//======================
	outdx = 0;
	outdy = 0;
	edysblm = 0;
	edxsblm = 0;
	pidx = 0;
	pidy= 0;
	//==================
	
	for(int i=0;i<=76;i++){
 	 sudutNgurang=2.4*i;
        x=-14;//+2*sin(sudutNgurang*3.14/180);
	y=-0.5-5*sin(sudutNgurang*3.14/180)- pidy;//2.4
        z=3*cos(sudutNgurang*3.14/180);
        sudutX=2-8*sin(sudutNgurang*3.14/180)- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=5+4*sin(sudutNgurang*3.14/180);//+sin(sudutNgurang*3.14/180);
	// speed=200*cos(sudutNgurang*3.14*0.5/180);
        sudutZ=0;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14+4*sin(sudutNgurang*3.14/180);
        y2=-0.5-6*sin(sudutNgurang*3.14/180);//2.4
        z2=-2*cos(sudutNgurang*3.14/180);
        sudutX2=-2-8*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=5+4*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	 speed=100;//cos(sudutNgurang*3.14/360);
	 //===============
	sensen();
	pidtangan();
	//===========
	inverseAndre(tkan,tki,sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
usleep(1000);
	
//	while(i>=26){
	//PID x
        // P x
        epx = setpointx - nilaisensor;
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
//	}
			}

	//usleep(100);
	// BALIKIN MIRING//
	
	for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*1.4;
        y=-0.5+0.5*sin(sudutNgurang*3.14/180);//2.4
        z=-3;//cos(sudutNgurang*3.14/180);
	sudutX=2-2*sin(sudutNgurang*3.14/180);
        sudutY=5;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=-0.5+0.5*sin(sudutNgurang*3.14/180);//2.4
        z2=2;//-0.2;//cos(sudutNgurang*3.14/180);
	sudutX2=-2+2*sin(sudutNgurang*3.14/180);
        sudutY2=5;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
         speed=100;//cos(sudutNgurang*3.14/360);
//=============
sensen();
	pidtangan();
//=====================	
	inverseAndre(tkan,tki,sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
usleep(2000);

		}
		usleep(1000);
   // MIRING //


	for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*2;
        y=1.5*sin(sudutNgurang*3.14/180);//2.4
        z=-3;//cos(sudutNgurang*3.14/180);
        sudutX=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=5+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=1.5*sin(sudutNgurang*3.14/180);//2.4
        z2=2;//-0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=5+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);
		//==============
		sensen();
	pidtangan();
	//=====================
        inverseAndre(tkan,tki,sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
usleep(2000);			}


   usleep(100);
        //ANGKAT KAKI DAN MENDARAT//
	
	//=======================
	outdx = 0;
	outdy = 0;
	edysblm = 0;
	edxsblm = 0;
	pidx = 0;
	pidy= 0;
    //=======================
		
		for(int i=0;i<=76;i++){
         sudutNgurang=2.4*i;
        x=-14+4*sin(sudutNgurang*3.14/180);//1.5;
        y=1.5+5*sin(sudutNgurang*3.14/180);//2.4
        z=-3*cos(sudutNgurang*3.14/180);
        sudutX=-2-8*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7+1*sin(sudutNgurang*3.14/180);
        sudutZ=0;//cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*1.5;
        y2=1.5+6*sin(sudutNgurang*3.14/180)+pidy;//2.4
        z2=2*cos(sudutNgurang*3.14/180);
        sudutX2=2-8*sin(sudutNgurang*3.14/180)- pidx;//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7+1*sin(sudutNgurang*3.14/180);//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//*cos(sudutNgurang*3.14/360);
		//==================
		sensen();
	pidtangan();
	//================
        inverseAndre(tkan,tki,sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
usleep(1000);
	 
//	while(i>=26){
	//PID x
		// P x
        epx = setpointx - nilaisensor;
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
//	}
			}

   usleep(100);
        // BALIKIN MIRING//

        for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*1.4;
        y=1.5-1.5*sin(sudutNgurang*3.14/180);//2.4
        z=3;//cos(sudutNgurang*3.14/180);
        sudutX=-2+2*sin(sudutNgurang*3.14/180);
        sudutY=7;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//-2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=1.5-1.5*sin(sudutNgurang*3.14/180);//2.4
        z2=-2;//0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=2-2*sin(sudutNgurang*3.14/180);
        sudutY2=7;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//*cos(sudutNgurang*3.14/360);
		//=================
		sensen();
	pidtangan();
	//====================
        inverseAndre(tkan,tki,sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	usleep(2000);

		}
	usleep(2000);
	}
	sleep(30000);
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

}
return 0;
}

#endif
