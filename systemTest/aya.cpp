//#ifndef AYA_C
//#define AYA_C

#include "/home/pi/systemTest/library/allLibtune.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

double nilaisensor,epx,edx,epy,edy,erry,outpx,outpy;
double kpx = 0;
double kdx = 0;//.005;;
double kpy = 0.5;
double kdy = 0;
double outdx = 0;
double outdy = 0;
double edysblm = 0;
double edxsblm = 0;
double pidx = 0;
double pidy= 0;
double setpointx = 270.5;
double setpointy = 1.1;
int servo7, servo8;


void sensen(){

	string status;
	enableIMU();
	nilaisensor=bacaACC3(Pacc_raw);
	//cout<<status<<endl;
	//bacaMAG(Pmag_raw, 1);
	//nilaisensor = AccXangle;
//cout<<"a  "<<nilaisensor<<endl;	
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
        x=-14;//-sin(sudutNgurang*3.14/180)*2;//max -15//-12.3
        y=0;
        z=0;
        sudutX=0;
        sudutY=12;
        sudutZ=0;
        x2=-14;//max -15//-12.3
        y2=0;
        z2=0;
        sudutX2=0;
        sudutY2=12;
        sudutZ2=0;
	servo7=358;
	servo8=676;
	dxl_write_word(3,30,300);
	dxl_write_word(4,30,744);
	// inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,servo7,servo8,'2');
	cout<<"Masukkan Speed: ";
	//cin>>speed;
	sleep(500);
		//kirimPacketInverse(x,y);

	//for(int i=1;i<=10;i++){
	// MIRING //
	
     	
	 for(int i=0;i<=38;i++){
       	//cout<<"Masukkan X: ";
	 sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=2.1*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=-5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=6;//+sin(sudutNgurang*3.14/180);
        sudutZ=0.5;//3*(pow(sin(sudutNgurang*3.14/180),2));
	x2=-14;//+sin(sudutNgurang*3.14/180)*2;
      	y2=2.1*sin(sudutNgurang*3.14/180);//2.4
      	z2=0;
        sudutX2=5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=6;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
	  speed=100;//cos(sudutNgurang*3.14/360);

	//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,servo7,servo8,'2');

	usleep(10000);
	}
	
	usleep(4000);	
	//  ANKAT KAKI DAN MENDARAT //

	 for(int i=0;i<=76;i++){

	
	 sudutNgurang=2.4*i;
	x=-14+2*sin(sudutNgurang*3.14/180);
        y=2.1+0*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
        z=0.25+0.25-0.5*cos(sudutNgurang*3.14/180);
	sudutX=-5.5-11*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	
	sudutY=6+pidx;//+4*cos(sudutNgurang*3.14/180);
        sudutZ=0;//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-2*sin(sudutNgurang*3.14/180);
      	y2=2.1+1*sin(sudutNgurang*3.14/180);// + pidy;//sin(sudutNgurang*3.14/180);//2.4
      	z2=-0.5+0.5*cos(sudutNgurang*3.14/180);
        sudutX2=5.5-16*sin(sudutNgurang*3.14/180);// - pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=6+pidx;//+4*sin(sudutNgurang*3.14/180);
    sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	speed=100;//cos(sudutNgurang*3.14/360);
	//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	//baca sensor
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,servo7,servo8,'2');
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

			pidx = (outpx + outdx);
			
			//PID y
			// P y
			erry = cos(epx) + sin(epx);
			epy = setpointy - erry;
			outpy = kpy*epy;
			// D y
			edy = epy - edysblm;
			outdy = kdy*edy;
			edysblm = epy;
			
			pidy = 300+(outpy + outdy);
			//dxl_write_word(19,32,500);
			//dxl_write_word(19,32,500);
		//dxl_write_word(3,30,pidy);
		//dxl_write_word(4,30,pidx);	
	usleep(10000);	}
	usleep(5000);
	// BALIKIN MIRING //

        for(int i=0;i<=38;i++){
	 sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=2.1-2.1*sin(sudutNgurang*3.14/180);//2.4
        z=0.5;
        sudutX=-5.5+5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=6;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*2;
        y2=2.1-2.1*sin(sudutNgurang*3.14/180);//2.4
        z2=-1;//0.2;//.2;
        sudutX2=5.5-5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=6;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2 
	speed=100;//cos(sudutNgurang*3.14/360);
//	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');     
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,servo7,servo8,'2');

usleep(1000);
}
	usleep(10000);
	// MIRING //	
for(int j=0;j<=76;j++){


	for(int i=0;i<=38;i++){
	 sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*2;
        y=-2.5*sin(sudutNgurang*3.14/180);//2.4
        z=0.5;//cos(sudutNgurang*3.14/180);
        sudutX=4*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=6;//+1*sin(sudutNgurang*3.14/180);
        sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=-2.5*sin(sudutNgurang*3.14/180);//2.4
        z2=-1;//0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=-4*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=6;//+1*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
         speed=100;//cos(sudutNgurang*3.14/360);
//	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');      
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,servo7,servo8,'2');
	usleep(1000);		}
        usleep(4000);
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
        x=-14+0*sin(sudutNgurang*3.14/180);
	y=-2.5-1*sin(sudutNgurang*3.14/180);//- pidy;//2.4
        z=0.5*cos(sudutNgurang*3.14/180);
        sudutX=4-22*sin(sudutNgurang*3.14/180);//- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=6+pidx;//3*sin(sudutNgurang*3.14/180);//+sin(sudutNgurang*3.14/180);
	// speed=200*cos(sudutNgurang*3.14*0.5/180);
        sudutZ=0;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14+2*sin(sudutNgurang*3.14/180);
        y2=-2.5-1*sin(sudutNgurang*3.14/180);//2.4
        z2=-1*cos(sudutNgurang*3.14/180);
        sudutX2=-4-6*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=6+pidx;//3*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	 speed=100;//cos(sudutNgurang*3.14/360);
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,servo7,servo8,'2');
//	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	sensen();
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
	usleep(2000);		}

	usleep(5000);
	// BALIKIN MIRING//
	
	for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*1.4;
        y=-2.5+2.5*sin(sudutNgurang*3.14/180);//2.4
        z=-0.5;//cos(sudutNgurang*3.14/180);
	sudutX=4-4*sin(sudutNgurang*3.14/180);
        sudutY=6;//-1*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=-2.5+2.5*sin(sudutNgurang*3.14/180);//2.4
        z2=1;//-0.2;//cos(sudutNgurang*3.14/180);
	sudutX2=-4+4*sin(sudutNgurang*3.14/180);
        sudutY2=6;//-1*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
         speed=100;//cos(sudutNgurang*3.14/360);
//	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,servo7,servo8,'2');
	usleep(1000);
		}
		usleep(10000);
   // MIRING //


	for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*2;
        y=2*sin(sudutNgurang*3.14/180);//2.4
        z=-0.5;//cos(sudutNgurang*3.14/180);
        sudutX=-4*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=6;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=2*sin(sudutNgurang*3.14/180);//2.4
        z2=1;//-0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=4*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=6;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,servo7,servo8,'2');
//	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	usleep(1000);		}

   usleep(5000);
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
        x=-14+1.2*sin(sudutNgurang*3.14/180);
        y=2+2*sin(sudutNgurang*3.14/180);//2.4
        z=-0.5*cos(sudutNgurang*3.14/180);
        sudutX=-4-12*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=6+0*sin(sudutNgurang*3.14/180);
        sudutZ=0;//cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*1.5;
        y2=2+2*sin(sudutNgurang*3.14/180);//+pidy;//2.4
        z2=1*cos(sudutNgurang*3.14/180);
        sudutX2=4-16*sin(sudutNgurang*3.14/180);//- pidx;//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=6+0*sin(sudutNgurang*3.14/180);//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//*cos(sudutNgurang*3.14/360);
//        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,servo7,servo8,'2');
	 sensen();
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
	cout<<"a  "<<nilaisensor<<endl;

	usleep(3000);	}	

   usleep(5000);
        // BALIKIN MIRING//

        for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*1.4;
        y=2-2*sin(sudutNgurang*3.14/180);//2.4
        z=0.5;//cos(sudutNgurang*3.14/180);
        sudutX=-4+4*sin(sudutNgurang*3.14/180);
        sudutY=6;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//-2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=2-2*sin(sudutNgurang*3.14/180);//2.4
        z2=-1;//0.2;//cos(sudutNgurang*3.14/180);
        sudutX=4-4*sin(sudutNgurang*3.14/180);
        sudutY2=6;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//*cos(sudutNgurang*3.14/360);
//        inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,servo7,servo8,'2');
	usleep(100);
		}
	usleep(20000);
	}
//	sleep(30000);
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
usleep(3000);
}
return 0;
}

//#endif
