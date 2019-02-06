#ifndef INVERSETESTA_C
#define INVERSETESTA_C

//#include "/home/pi/systemTest/library/allLib.h"
////#include "/home/pi/systemTest/library/movement/moveArray.h"
////#include "/home/pi/systemTest/library/movement/moveArray.h"
//#include "/home/pi/systemTest/library/movement/moveArray.h"
//#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
int speed;

int gerak(){

//if(dxl_initialize(0,1)==0){
 //       cout<<"Error connecting"<<endl;
 //       return 0;
 //   }
    
//else{
//		cout<<"connect success!"<<endl;
//	}



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
        x=-10;//-sin(sudutNgurang*3.14/180)*2;
        y=0;
        z=0;
        sudutX=0;
        sudutY=8;
        sudutZ=0;
        x2=-10;
        y2=0;
        z2=0;
        sudutX2=0;
        sudutY2=8;
        sudutZ2=0;

	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
	cout<<"Masukkan Speed: ";
	//cin>>speed;
	sleep(5);
		//kirimPacketInverse(x,y);

	//for(int i=1;i<=10;i++){
	// MIRING //
	}
void miringKanan(double h,double miring  ,double f, double  yaw, double roll, double pitch)
{
	for(int i=0;i<=38;i++){
		sudutNgurang=2.4*i;
		x = hki;//-sin(sudutNgurang*3.14/180)*2;
		y = miringki*sin(sudutNgurang*3.14/180);//2.4
		z = fki;
		sudutX = yawki*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY = rollki;//+sin(sudutNgurang*3.14/180);
		sudutZ = pitchki;//3*(pow(sin(sudutNgurang*3.14/180),2));
		x2 = h;//+sin(sudutNgurang*3.14/180)*2;
		y2 = miring*sin(sudutNgurang*3.14/180);//2.4
		z2 = 0;
		sudutX2 = yaw*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY2 = roll;//-sin(sudutNgurang*3.14/180);
		sudutZ2 = pitch;//-3*(pow(sin(sudutNgurang*3.14/180),2));

		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,'2');
//	sleep(1);
	}
	usleep(100);
}

//  ANKAT KAKI DAN MENDARAT //
void angkatKaki_kiri(double h2,double s2  ,double f2, double  yaw2, double roll2, double pitch2)
{
	for(int i=0;i<=76;i++){
		sudutNgurang = 2.4*i;
		x = 10-h2kisin(sudutNgurang*3.14/180)*1.4;
		y = miring2ki;//cos(sudutNgurang*3.14/360);//2.4
		z = -f2ki*0.5+f2ki*0.5*cos(sudutNgurang*3.14/180);
		sudutX = 1.5-7*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	
		sudutY = 8;//+sin(sudutNgurang*3.14/180);
		sudutZ = 0;//-1+1*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
		x2 = -10+sin(sudutNgurang*3.14/180)*1.7;
		y2 = -1;//sin(sudutNgurang*3.14/180);//2.4
		z2 = 0.5-0.5*cos(sudutNgurang*3.14/180);
		sudutX2 = 1;//8*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY2 = 8+1*sin(sudutNgurang*3.14/180);
		sudutZ2 = 0;//-0.5+0.5*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,'2');
	}
	usleep(100);
}	// BALIKIN MIRING //
void balikinMiring_kanan(double h3,double s3  ,double f3, double  yaw3, double roll3, double pitch3)
{
	for(int i=0;i<=38;i++){
		sudutNgurang = 2.4*i;
		x = -10;//-sin(sudutNgurang*3.14/180)*2;
		y = -1+1*sin(sudutNgurang*3.14/180);//2.4
		z = -1;
		sudutX = 1.5-1.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY = 8;//+sin(sudutNgurang*3.14/180);
		sudutZ = 0;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
		x2 = -10;//+sin(sudutNgurang*3.14/180)*2;
		y2 = -1+1*sin(sudutNgurang*3.14/180);//2.4
		z2 = 1;//.2;
		sudutX2 = 1.5-1.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY2 = 8;//-sin(sudutNgurang*3.14/180);
		sudutZ2 = 0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2 
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,'2');     
	 }
	usleep(100);
	// MIRING //
}	
//for(int j=0;j<=7;j++){
void miringkiri_jalan(double h4,double s4  ,double f4, double  yaw4, double roll4, double pitch4){
	for(int i=0;i<=38;i++){
		sudutNgurang = 2.4*i;
		x = -10;//+sin(sudutNgurang*3.14/180)*2;
		y = 1*sin(sudutNgurang*3.14/180);//2.4
		z = -0.8;//cos(sudutNgurang*3.14/180);
		sudutX = -1.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY = 8;//+sin(sudutNgurang*3.14/180);
		sudutZ = 0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
		x2 = -10;//-sin(sudutNgurang*3.14/180)*2.4;
		y2 = 1*sin(sudutNgurang*3.14/180);//2.4
		z2 = 1;//cos(sudutNgurang*3.14/180);
		sudutX2 = -1.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY2 = 8;//-sin(sudutNgurang*3.14/180);
		sudutZ2 = 0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,'2');      
	}
        usleep(100);
}
//ANGKAT KAKI DAN MENDARAT//
void angakatKaki_kanan_jalan(double h5,double s5  ,double f5, double  yaw5, double roll5, double pitch5){
	for(int i=0;i<=76;i++){
 		sudutNgurang = 2.4*i;
		x = -10+sin(sudutNgurang*3.14/180)*3;
		y = 1;//sin(sudutNgurang*3.14/180);//2.4
		z = -1*cos(sudutNgurang*3.14/180);
		sudutX = -1.8;//-8*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY = 8+1*sin(sudutNgurang*3.14/180);//+sin(sudutNgurang*3.14/180);
		sudutZ = 0;//-1-1*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
		x2 = -10-sin(sudutNgurang*3.14/180)*1.5;
		y2 = 1;//sin(sudutNgurang*3.14/180);//2.4
		z2 = 1*cos(sudutNgurang*3.14/180);
		sudutX2 = -1.5+8*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY2 = 8;//-sin(sudutNgurang*3.14/180);
		sudutZ2 = 0;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,'2');
	}
	usleep(100);
}
	// BALIKIN MIRING//}
void balikinmiring_kiri_jalan(double h6,double s6  ,double f6, double  yaw6, double roll6, double pitch6){
	for(int i=0;i<=38;i++){
        	sudutNgurang = 2.4*i;
		x = -10;//+sin(sudutNgurang*3.14/180)*1.4;
		y = 1-1*sin(sudutNgurang*3.14/180);//2.4
		z = 1;//cos(sudutNgurang*3.14/180);
		sudutX = -1.5+1.5*sin(sudutNgurang*3.14/180);
		sudutY = 8;//+sin(sudutNgurang*3.14/180);
		sudutZ = 0;//3*(pow(sin(sudutNgurang*3.14/180),2));
		x2 = -10;//-sin(sudutNgurang*3.14/180)*1.4;
		y2 = 1-1*sin(sudutNgurang*3.14/180);//2.4
		z2 = -1;//cos(sudutNgurang*3.14/180);
		sudutX2 = -1.5+1.5*sin(sudutNgurang*3.14/180);
		sudutY2=8;//-sin(sudutNgurang*3.14/180);
		sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,'2');
	}
	usleep(100);
   // MIRING //
}
void miringkanan_jalan(double h7,double s7  ,double f7, double  yaw7, double roll7, double pitch7){
	for(int i=0;i<=38;i++){
        	sudutNgurang = 2.4*i;
		x = -10;//+sin(sudutNgurang*3.14/180)*2;
		y = -1*sin(sudutNgurang*3.14/180);//2.4
		z = 1;//cos(sudutNgurang*3.14/180);
		sudutX = 1.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY = 8;//+sin(sudutNgurang*3.14/180);
		sudutZ = 0;//3*(pow(sin(sudutNgurang*3.14/180),2));
		x2 = -10;//-sin(sudutNgurang*3.14/180)*2.4;
		y2 = -1*sin(sudutNgurang*3.14/180);//2.4
		z2 = -1;//cos(sudutNgurang*3.14/180);
		sudutX2 = 1.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY2 = 8;//-sin(sudutNgurang*3.14/180);
		sudutZ2 = 0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,'2');
	}
	usleep(100);
}

//ANGKAT KAKI DAN MENDARAT//
void angkatkaki_kiri_jalan(double h8,double s8  ,double f8, double  yaw8, double roll8, double pitch8){
        for(int i=0;i<=76;i++){
        	sudutNgurang = 2.4*i;
		x = -10-sin(sudutNgurang*3.14/180)*1.5;
		y = -1;//sin(sudutNgurang*3.14/180);//2.4
		z = 1*cos(sudutNgurang*3.14/180);
		sudutX = 1.5-8*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY = 8;//+sin(sudutNgurang*3.14/180);
		sudutZ = 0;//-1+1*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
		x2 = -10+sin(sudutNgurang*3.14/180)*1.5;
		y2 = -1;//sin(sudutNgurang*3.14/180);//2.4
		z2 = -1*cos(sudutNgurang*3.14/180);
		sudutX2 = 1.5;//+6*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
		sudutY2 = 8+1*sin(sudutNgurang*3.14/180);//-sin(sudutNgurang*3.14/180);
		sudutZ2 = 0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,'2');
	}
	usleep(100);
}

// BALIKIN MIRING//
void balikinmiring_kanan_jalan(double h9,double s9  ,double f9, double  yaw9, double roll9, double pitch9){
        for(int i=0;i<=38;i++){
        	sudutNgurang = 2.4*i;
		x = -10;//+sin(sudutNgurang*3.14/180)*1.4;
		y = -1+1*sin(sudutNgurang*3.14/180);//2.4
		z = -1;//cos(sudutNgurang*3.14/180);
		sudutX = 1.5-1.5*sin(sudutNgurang*3.14/180);
		sudutY = 8;//+sin(sudutNgurang*3.14/180);
		sudutZ = 0;//-2;//3*(pow(sin(sudutNgurang*3.14/180),2));
		x2 = -10;//-sin(sudutNgurang*3.14/180)*1.4;
		y2 = -1+1*sin(sudutNgurang*3.14/180);//2.4
		z2 = 1;//cos(sudutNgurang*3.14/180);
		sudutX = 1.5-1.5*sin(sudutNgurang*3.14/180);
		sudutY2 = 8;//-sin(sudutNgurang*3.14/180);
		sudutZ2 = 0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,'2');
	}
	usleep(100);
	sleep(3);
}

#endif
