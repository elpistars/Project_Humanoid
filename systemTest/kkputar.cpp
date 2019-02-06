	
//#ifndef AYANYAN_C 
//#define AYANYAN_C

#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

float nilaisensor,nilaisensor2;
double epka,epki,edka,edki,epx,edx,epy,edy,erry,outpx,outpy,outpka,outpki,outdka,outdki,outdx,outdy,outpy2,outdy2,epy2,edy2,torka ;
double kpx = 0;//.025; //pid kanan kiri
double kdx = 0;//4.23;//.4;	//pid kanan kiri
	double kpy = 0;//20.25;//.014;	//pid kanan kiri
	double kdy = 0;//15.0004;//.23;//.0068;	//pid kanan kiri
double kpka = 0;// kp tangan kanan
double kdka = 0; // kd tangan kanan
double kpki = 0; // kp tangan kiri
double kdki = 0; // kd tangan kiri
double kpy2 = 0.155;	//pid depan belakang
double kdy2 = 0.05;	//pid depan belakang
double edysblm = 0; //pid kanan kiri
double edkasblm = 0;	//pid tangan
double edkisblm = 0;	//pid tangan
double edxsblm = 0; //pid kanan kiri
double pidx = 0;
double pidy= 0;
double setpointx = 184;	//pid kanankiri dan tangan
double setpointy = 1;//1.1;
double pidka = 0;
double pidki = 0;
double setpointy2 = 270; // pid depan belakang
double pidy2 = 0;
double edy2sblm = 0;	//pid depan belakang
int tkan,tki;


void pidtangan(){
	
		//PID tangan ka
			// P ka
			epka = setpointx - nilaisensor;
			outpka = kpka*epka;
			//D ka
			edka = epka - edkasblm;
			outdka = kdka*edka;
			edkasblm = epka;

			pidka = outpka + outdka;
			tkan = 300 + pidka;
			//PID tangan ki
			// P ki
			epki = setpointx - nilaisensor;
			outpki = kpki*epki;
			//D ki
			edki = epki - edkisblm;
			outdki = kdki*edki;
			edkisblm = epki;

			pidki = outpki + outdki;
			tki = 744 + pidki;
}

void pidkanankiri(){
	
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

void piddepanbelakang(){
	//PID y
			// P y
			epy2 = setpointy2 - nilaisensor2;
			outpy2 = kpy2*epy2;
			//D y
			edy2 = epy2 - edy2sblm;
			outdy2 = kdy2*edy2;
			edy2sblm = epy2;

			pidy2 = outpy2 + outdy2;
	
}

void geraktangan(){	
	int id1[] = {3,4};
	int data1[] = {tkan,tki};
	setAllSpeed(100);
    kirimPacketGerak1(2, 1, 1, id1, data1, 1, 1000);
}

void sensen(){

	enableIMU();
	nilaisensor = bacaACC2(Pacc_raw);
	nilaisensor2 = bacaACC3(Pacc_raw);
	
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
        y=0;//4;
        z=0;
        sudutX=0;//-10;
        sudutY=10;
        sudutZ=0;
        x2=-14;
        y2=0;//4;
        z2=0;
        sudutX2=0;//-12;
        sudutY2=10;
        sudutZ2=0;
	//============
	sensen();
	
	tkan = 300;
	tki = 744;
	//===============
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,300,300,'2');
	cout<<"Masukkan Speed: ";
	geraktangan();
	//cin>>speed;
	/*cout<<"tkan : "<<tkan<<endl;
	cout<<"tki : "<<tki<<endl;
 cout<<"pidka : "<<pidka<<endl;
        cout<<"pidki : "<<pidki<<endl;
 cout<<"nilaisensor : "<<nilaisensor<<endl;
        */
	usleep(1000000);
		//kirimPacketInverse(x,y);
//}
	//for(int i=1;i<=10;i++){
	// MIRING //
//
//==================================
	edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;
 //============================
 
	 for(int i=0;i<=38;i++){
       	//cout<<"Masukkan X: ";
	 sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=-2*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=10+pidy2;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
	x2=-14;//+sin(sudutNgurang*3.14/180)*2;
      	y2=-2*sin(sudutNgurang*3.14/180);//2.4
      	z2=0;
        sudutX2=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10+pidy2;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
		  speed=100;//cos(sudutNgurang*3.14/360);
	  sensen();
	 		        
         epx = setpointx - nilaisensor;
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
                torka=1000+pidy*0.01/0.0014667;
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
		y=y+0.05*epy;
		y2=y2+0.05*epy;	
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,torka,torka,'2');
	usleep(1000);
	//piddepanbelakang();
	//pidtangan();
//	geraktangan();
	}
	
	usleep(100);	
	//  ANKAT KAKI DAN MENDARAT //

	//====================
	edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;
//====================

	 for(int i=0;i<=76;i++){

	
	 sudutNgurang=2.4*i;
	x=-14;//+2*sin(sudutNgurang*3.14/180)*1.4;
        y=-2+0*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
        z=0;//-0.5+0.5*cos(sudutNgurang*3.14/180);
	sudutX=2-10*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	
	sudutY=10+3*cos(sudutNgurang*3.14/180);//+pidy2;
        sudutZ=-5+5*cos(sudutNgurang*3.14/180);//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14+2*sin(sudutNgurang*3.14/180);
      	y2=-2+0*sin(sudutNgurang*3.14/180) ;//+ pidy;//sin(sudutNgurang*3.14/180);//2.4
      	z2=0;//0.5-0.5*cos(sudutNgurang*3.14/180);
        sudutX2=-2-16*sin(sudutNgurang*3.14/180) - pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10+3*sin(sudutNgurang*3.14/360);//+pidy2;
    sudutZ2=-5+5*cos(sudutNgurang*3.14/180);//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	speed=100;//cos(sudutNgurang*3.14/360);
	//============
	sensen();
			
	//============
	  epx = setpointx - nilaisensor;
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
                torka=1000+pidy*0.01/0.0014667;
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
		y=y+0.05*epy;
		y2=y2+0.05*epy;	
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,torka,torka,'2');
	//baca sensor
	


	usleep(3000);
//	piddepanbelakang();
//	pidtangan();
//	geraktangan();
	}
	//usleep(70);
	
	usleep(100);
	
	// BALIKIN MIRING //
//===============
	edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;
//======================	
        for(int i=0;i<=38;i++){
	 sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=-2+2*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=2-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=10+pidy2;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=-10;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*2;
        y2=-2+2*sin(sudutNgurang*3.14/180);//2.4
        z2=0;//0.2;//.2;
        sudutX2=-2+2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10+pidy2;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=-10;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2 
	speed=100;//cos(sudutNgurang*3.14/360);
	//=============
	sensen();
	
	//====================
	  epx = setpointx - nilaisensor;
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
                torka=300+pidy*0.01/0.0014667;
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
		y=y+0.05*epy;
		y2=y2+0.05*epy;	
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,torka,torka,'2');
	usleep(100);
	//piddepanbelakang();
	//pidtangan();
	//geraktangan();
}	
	usleep(100);
	// MIRING //	

//================
	edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;
//==================	
	for(int i=0;i<=38;i++){
	 sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*2;
        y=3*sin(sudutNgurang*3.14/180);//2.4
        z=0;//cos(sudutNgurang*3.14/180);
        sudutX=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=10-0*sin(sudutNgurang*3.14/180)+pidy2;

        sudutZ=-10;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=3*sin(sudutNgurang*3.14/180);//2.4
        z2=0;//0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10-0*sin(sudutNgurang*3.14/180)+pidy2;
        sudutZ2=-10;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
         speed=50;//cos(sudutNgurang*3.14/360);
		 //================
		 sensen();
	
	//================
	   epx = setpointx - nilaisensor;
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
                torka=300+pidy*0.01/0.0014667;
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
		y=y+0.05*epy;
		y2=y2+0.05*epy;	
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,torka,torka,'2');
	usleep(2000);
	
	//piddepanbelakang();
	//pidtangan();
	//geraktangan();
		}
        usleep(1000);
	//ANGKAT KAKI DAN MENDARAT/
	
	//======================
	//pidy2 = 0;
	//edy2sblm = 0;
	//edysblm = 0;
	edxsblm = 0;
	pidx = 0;
	//pidy= 0;
	edkasblm = 0;
	edkisblm = 0;
	//==================
	
	for(int i=0;i<=76;i++){
 	 sudutNgurang=2.4*i;
        x=-14+2*sin(sudutNgurang*3.14/180);
	y=3+0*sin(sudutNgurang*3.14/180)- pidy;//2.4
        z=0*cos(sudutNgurang*3.14/360);
        sudutX=-2-16*sin(sudutNgurang*3.14/180)- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=10-10*sin(sudutNgurang*3.14/180)+pidy2;//+sin(sudutNgurang*3.14/180);
	// speed=200*cos(sudutNgurang*3.14*0.5/180);
        sudutZ=-10+5-5*cos(sudutNgurang*3.14/180);//8*sin(sudutNgurang*3.14/360);//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+2*sin(sudutNgurang*3.14/180);
        y2=3+0*sin(sudutNgurang*3.14/180);//2.4
        z2=0*cos(sudutNgurang*3.14/180);
        sudutX2=2-10*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10-10*sin(sudutNgurang*3.14/180)+pidy2;
        sudutZ2=-10+5-5*cos(sudutNgurang*3.14/180);//8*sin(sudutNgurang*3.14/360);//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	 speed=100;//cos(sudutNgurang*3.14/360);
	 //===============
	sensen();
	
	//===========
	  epx = setpointx - nilaisensor;
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
                torka=300+pidy*0.01/0.0014667;
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
		y=y+0.05*epy;
		y2=y2+0.05*epy;	
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,torka,torka,'2');
	usleep(3000);
//	piddepanbelakang();
//	pidtangan();
//	geraktangan();


	//pidkanankiri();


			}

	usleep(100);
	// BALIKIN MIRING//
//=======================
//		pidy2 = 0;
///		edy2sblm = 0;
	edkasblm = 0;
	edkisblm = 0;
//==========================
	
	for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*1.4;
        y=3-3*sin(sudutNgurang*3.14/180);//2.4
        z=0;//cos(sudutNgurang*3.14/180);
	sudutX=-2+2*sin(sudutNgurang*3.14/180);
        sudutY=10+pidy2;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=3-3*sin(sudutNgurang*3.14/180);//2.4
        z2=0;//-0.2;//cos(sudutNgurang*3.14/180);
	sudutX2=2-2*sin(sudutNgurang*3.14/180);
        sudutY2=10+pidy2;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
         speed=100;//cos(sudutNgurang*3.14/360);
//=============
	sensen();
	
//=====================	
	  epx = setpointx - nilaisensor;
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
                torka=300+pidy*0.01/0.0014667;
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
		y=y+0.05*epy;
		y2=y2+0.05*epy;	
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,torka,torka,'2');
	usleep(3000);
	//piddepanbelakang();
	//pidtangan();
//	geraktangan();
		}
		//sleep(100);
		sensen();
	
	//================
         epx = setpointx - nilaisensor;
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
                torka=300+pidy*0.01/0.0014667;
                	

	
			

   
        // BALIKIN MIRING//
//============	
	//pidy2 = 0;
	//edy2sblm = 0;
	//e/dkasblm = 0;
	//edkisblm = 0;
  //=================  
    

	usleep(1000);
}


int main(){

//if(dxl_initialize(1,1)==0){
 //       cout<<"Error connecting"<<endl;
 //       return 0;
  //  }
    
//else{
//		cout<<"connect success!"<<endl;
//	}
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

//#endif
