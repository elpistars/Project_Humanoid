#ifndef BTRY1_CPP
#define BTRY1_CPP

#include "/home/pi/systemTest/library/allLibtune.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

using namespace std;

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

string * MenuItems(){
	string *item=new string[7];
	item[0]="Tuning Berjalan";
	item[1]="Tuning Berdiri";
	item[2]="Tuning Muter Kiri";
	item[3]="Tuning Muter Kanan";
	item[4]="Tuning Bangun";
	item[5]="Start ALL Motion";
	item[6]="Exit.";

	return item;
}
bool status;

/*void gotoxy(int xpos, int ypos){
	/*COORD scrn;    
	HANDLE hOuput = GetStdHandle(STD_OUTPUT_HANDLE); 
	scrn.X = xpos; scrn.Y = ypos;
	SetConsoleCursorPosition(hOuput,scrn);

	printf("%c[%d;%df",0x1B,ypos,xpos);
}*/

/*void ChangeCursorStatus(bool){
	CONSOLE_CURSOR_INFO *c=new CONSOLE_CURSOR_INFO;
	HANDLE h=GetStdHandle(STD_OUTPUT_HANDLE);

	if (Visible)
	{
		c->bVisible=TRUE;
		c->dwSize=0;//(0) is invild so the default value is set
	}
	else
	{
		c->bVisible=FALSE;
		c->dwSize=1;//it must be assigned to a valid value
	}
	
	SetConsoleCursorInfo(h,c);
}*/

void MenuTuning1(){
	int x1, x2, y1, y2, z1, z2, Sdx1, Sdy1, Sdz1, Sdx2, Sdy2, Sdz2;
	char a;
	system("clear");

	while(status){
		gotoxy(0,0); cout << "Console Tuning Berjalan" << endl << endl;

		cout << "Nilai x kiri        : "; cin >> x1;
		cout << "Nilai y kiri        : "; cin >> y1;
		cout << "Nilai z kiri        : "; cin >> z1;
		cout << "Nilai Sudut x kiri  : "; cin >> Sdx1;
		cout << "Nilai Sudut y kiri  : "; cin >> Sdy1;
		cout << "Nilai Sudut z kiri  : "; cin >> Sdz1;

		cout << "Nilai x kanan       : "; cin >> x2;
		cout << "Nilai y kanan       : "; cin >> y2;
		cout << "Nilai z kanan       : "; cin >> z2;
		cout << "Nilai Sudut x kanan : "; cin >> Sdx2;
		cout << "Nilai Sudut y kanan : "; cin >> Sdy2;
		cout << "Nilai Sudut z kanan : "; cin >> Sdz2;
		
		cout << endl;
		cout << "Data Sudah benar? Press ENTER jika iya. Tekan C jika ingin mengulang." << endl;
		cout << "Tekan apa saja jika ingin kembali ke menu awal" << endl;

		getch();

		a = toupper(getch());
	
		switch(a){
		case 'X':
			cout << endl << endl; 
			cout << "Data Sudah Dikirim! Tekan key apa saja.";
			getch();
			status=false;
			system("clear");
			break;
		case 'C':
			status = true;
			system("clear");
			break;
		default:
			try{
				status=false;
				system("clear");
			}catch(...){
			}
		}
	}
}
//-----------------------------------------------------------------------------------------------
void MenuTuning2(){
	system("clear");
	gotoxy(2,2);
	cout<<" TUNING BANGUN "<<endl;
	getch();
	system("clear");
}
void MenuTuning3(){
	system("clear");
	gotoxy(2,2);
	cout<<" PUTER KIRI "<<endl;
	getch();
	system("clear");
}
void MenuTuning4(){
	system("clear");
	gotoxy(2,2);
	cout<<" PUTER KANAN "<<endl;
	getch();
	system("clear");
}
void MenuTuning5(){
	system("clear");
	gotoxy(2,2);
	cout<<" TUNING BANGUN "<<endl;
	getch();
	system("clear");
}
void MenuTuning6(){
	system("clear");
	gotoxy(2,2);
	cout<<" START ALL MOTION "<<endl;
	getch();
	system("clear");
}
void ExitOpt(){
	gotoxy(2,10);
	cout<<"Exitting..."<<endl;
	getch();
	exit(0);
}

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
	int speed, com7, com8;


//	com7=4;
//  	com8=4;
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
	cin>>x;
	cout<<"Masukkan Y: ";
	cin>>y;
	cout<<"Masukkan Z: ";
	cin>>z;*/
	x=-11.4;// tinggi kaki
	y=0;// miring kaki
	z=0;// langkah kaki
	sudutX=0;//miring badan
	sudutY=-3;//kebungkukan badan
	sudutZ=0;//putaran badan
	x2=-11.4;
	y2=0;//3;
	z2=0;
	sudutX2=0;//-6;
	sudutY2=-3;
	sudutZ2=0;
	dxl_write_word(3,30,300);
	dxl_write_word(4,30,724);
	// inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,100,512,512,'2');
	cout << "Masukkan Speed: ";
	//cin>>speed;
	sleep(5);
	
	//kirimPacketInverse(x,y);
	//sleep(1000);
	//for(int i=1;i<=10;i++){
	// MIRING //
	f2=0;
    while(f2<=90.0)
	//for(int i=0;i<=38;i++)
   {
	f2=f2+a;
       	//cout<<"Masukkan X: ";
	sudutNgurang=f2;
        x=-11.4;//-2*sin(sudutNgurang*3.14/180)*2;
        y=-1*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=-3;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
	x2=-11.4;//+sin(sudutNgurang*3.14/180)*2;
      	y2=-1*sin(sudutNgurang*3.14/180);//2.4
      	z2=0;
        sudutX2=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=-3;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
	speed=50;//cos(sudutNgurang*3.14/360);
	//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
	//usleep(1000);
	//cout<<"Masukkan sudut X: "<<f2<<endl;
   }
	f2=0;
	//usleep(1000);	
	
// ------------------  ANKAT KAKI DAN MENDARAT //
	cout << "Masukkan sudut X: ";
	//for(int i=0;i<=76;i++)
	while(f2<=180)
	{
		f2=f2+a;
		sudutNgurang=f2;
		x=-11.4+3*sin(sudutNgurang*3.14/180);
        	y=-1-0*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
	        z=0+0.8-0.8*cos(sudutNgurang*3.14/180);
		sudutX=2+8*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	
		sudutY=-3;//-2*sin(sudutNgurang*3.14/360);
	        sudutZ=0;//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
	        x2=-11.4-0*sin(sudutNgurang*3.14/180);
	      	y2=-1-2*sin(sudutNgurang*3.14/180);// + pidy;//sin(sudutNgurang*3.14/180);//2.4
	      	z2=-0.8+0.8*cos(sudutNgurang*3.14/180);
	        sudutX2=-2+16*sin(sudutNgurang*3.14/180);// - pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
	        sudutY2=-3-2*sin(sudutNgurang*3.14/360);
		sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
		speed=100;//+50*cos(sudutNgurang*3.14/360);
		//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	
	//baca sensor
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
	
	//sensen();
	//if(i>46){
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
	//usleep(1000);	
	}
	//usleep(1000);
	
	f2=0;
	//cout<<"Masukkan sudut X: ";
	
// --------------------------- BALIKIN MIRING //

        //for(int i=0;i<=38;i++)
	while(f2<=90)
	{
		f2=f2+a;
		sudutNgurang=f2;
	        x=-11.4;//-sin(sudutNgurang*3.14/180)*2;
	        y=-1+1*sin(sudutNgurang*3.14/180);//2.4
	        z=1.6;
	        sudutX=2-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
	        sudutY=-3;//+2*sin(sudutNgurang*3.14/180);
	        sudutZ=0;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
	        x2=-11.4;//+sin(sudutNgurang*3.14/180)*2;
	        y2=-1+1*sin(sudutNgurang*3.14/180);//2.4
	        z2=-1.6;//0.2;//.2;
	        sudutX2=-2+2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
	        sudutY2=-5+2*sin(sudutNgurang*3.14/180);
	        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2 
		speed=100;//cos(sudutNgurang*3.14/360);
		//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');     
		inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
		//usleep(1000);
		//cout<<"Masukkan sudut X: ";
	}
	f2=0;
	
	//cout<<"Masukkan sudut X: ";	
	
	usleep(10000);

// ----------------------------- MIRING //	
	for(int j=0;j<=76;j++){
		while(f2<=90)
		{
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180)*2;
			y=1*sin(sudutNgurang*3.14/180);//2.4
			z=1.2;//0.5;//cos(sudutNgurang*3.14/180);
			sudutX=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY=-3;//+1*sin(sudutNgurang*3.14/180);
			sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*2.4;
			y2=1*sin(sudutNgurang*3.14/180);//2.4
			z2=-1.2;//0.2;//cos(sudutNgurang*3.14/180);
			sudutX2=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=-3;//+1*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');      
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
		}
		//usleep(50000);
	
// ------------------------ ANGKAT KAKI DAN MENDARAT/
		f2=0;
		
		//======================
		outdx = 0;
		outdy = 0;
		edysblm = 0;
		edxsblm = 0;
		pidx = 0;
		pidy= 0;
		//==================
		
		while(f2<=180){
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4+0*sin(sudutNgurang*3.14/180);
			y=1+2*sin(sudutNgurang*3.14/180);//- pidy;//2.4
			z=1.2*cos(sudutNgurang*3.14/180);
			sudutX=-2+12*sin(sudutNgurang*3.14/180);//- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY=-3+2*sin(sudutNgurang*3.14/360);//+sin(sudutNgurang*3.14/180);
			//speed=200*cos(sudutNgurang*3.14*0.5/180);
			sudutZ=0;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4+2*sin(sudutNgurang*3.14/180);
			y2=1+0*sin(sudutNgurang*3.14/180);//2.4
			z2=-1.2*cos(sudutNgurang*3.14/180);
			sudutX2=2+16*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=-3;//+2*sin(sudutNgurang*3.14/360);
			sudutZ2=0;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//+50*sin(sudutNgurang*3.14/180);
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
		}
		//usleep(2000);
		f2=0;
		// BALIKIN MIRING//
		
		while(f2<=90){
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180)*1.4;
			y=1-1*sin(sudutNgurang*3.14/180);//2.4
			z=-1.2;//cos(sudutNgurang*3.14/180);
			sudutX=-2+2*sin(sudutNgurang*3.14/180);
			sudutY=-1-2*sin(sudutNgurang*3.14/180);
			sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*1.4;
			y2=1-1*sin(sudutNgurang*3.14/180);//2.4
			z2=1.2;//cos(sudutNgurang*3.14/180);
			sudutX2=2-2*sin(sudutNgurang*3.14/180);
			sudutY2=-3;//+2*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
		}
		
		f2=0;
		//usleep(500000);
		usleep(100000);

	// ---------------------------- MIRING //
		while(f2<=90){
			f2=f2+a; 
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180)*2;
			y=-1*sin(sudutNgurang*3.14/180);//2.4
			z=-1.2;//cos(sudutNgurang*3.14/180);
			sudutX=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY=-3;//+2*sin(sudutNgurang*3.14/180);
			sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*2.4;
			y2=-1*sin(sudutNgurang*3.14/180);//2.4
			z2=1.2;//-0.2;//cos(sudutNgurang*3.14/180);
			sudutX2=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=-3;//+2*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//cos(sudutNgurang*3.14/360);
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
		}

		//usleep(5000);
		
	// ------------------------ ANGKAT KAKI DAN MENDARAT//
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
			x=-11.4+2*sin(sudutNgurang*3.14/180);
			y=-1-0*sin(sudutNgurang*3.14/180);//2.4
			z=-1.2*cos(sudutNgurang*3.14/180);
			sudutX=2+14*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY=-3;//+2*sin(sudutNgurang*3.14/360);
			sudutZ=0;//cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//+sin(sudutNgurang*3.14/180)*1.5;
			y2=-1-2*sin(sudutNgurang*3.14/180);//+pidy;//2.4
			z2=1.2*cos(sudutNgurang*3.14/180);
			sudutX2=-2+16*sin(sudutNgurang*3.14/180);//- pidx;//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=-3+2*sin(sudutNgurang*3.14/360);//-sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//1+100*cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
		
		//sensen();
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
			//cout<<"a  "<<nilaisensor<<endl;

		}	
		f2=0;
		
		//usleep(5000);
		
	// ---------------------- BALIKIN MIRING//
		while(f2<=90){
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180)*1.4;
			y=-1+1*sin(sudutNgurang*3.14/180);//2.4
			z=1.2;//cos(sudutNgurang*3.14/180);
			sudutX=2-2*sin(sudutNgurang*3.14/180);
			sudutY=-3;//-2*sin(sudutNgurang*3.14/180);
			sudutZ=0;//-2;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*1.4;
			y2=-1+1*sin(sudutNgurang*3.14/180);//2.4
			z2=-1.2;//0.2;//cos(sudutNgurang*3.14/180);
			sudutX2=-2+2*sin(sudutNgurang*3.14/180);
			sudutY2=-1-2*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-3(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
		}
		
		usleep(100000);
		f2=0;
		while(f2<=90)
		{
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180);
			y=1*sin(sudutNgurang*3.14/180);//2.4
			z=1.2;//0.5;//cos(sudutNgurang*3.14/180);
			sudutX=-2*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
			sudutY=-3;//+1*sin(sudutNgurang*3.14/180);
			sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4*sin(sudutNgurang*3.14/180)*2.4;
			y2=1*sin(sudutNgurang*3.14/180);//2.4
			z2=-1.2;//0.2;//cos(sudutNgurang*3.14/180);
			sudutX2=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=-3;//+1*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
		}
		
		f2=0;
		while(f2<=180)
		{
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180);
			y=1+3*sin(sudutNgurang*3.14/360);//2.4
			z=1.2-1.2*sin(sudutNgurang*3.14/180);
			sudutX=-2+12*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
			sudutY=-3+2*sin(sudutNgurang*3.14/180);
			sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4+1.5*sin(sudutNgurang*3.14/180)*2.4;
			y2=1;//sin(sudutNgurang*3.14/180);//2.4
			z2=-1.2+2*sin(sudutNgurang*3.14/360);
			sudutX2=2+18*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=-3;//+1*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
		}
		
		f2=0;
		while(f2<=90)
		{
			f2=f2+a;
			sudutNgurang=f2;
			x=-11.4;//+sin(sudutNgurang*3.14/180);
			y=1-1*sin(sudutNgurang*3.14/180);//2.4
			z=0;//0.5;//cos(sudutNgurang*3.14/180);
			sudutX=-2+2*sin(sudutNgurang*3.14/180);//(pow(sin(sudutNgurang*3.14/180),2));
			sudutY=-3;//+1*sin(sudutNgurang*3.14/180);
			sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
			x2=-11.4;//-sin(sudutNgurang*3.14/180)*2.4;
			y2=1-1*sin(sudutNgurang*3.14/180);//2.4
			z2=0.8;//0.2;//cos(sudutNgurang*3.14/180);
			sudutX2=2-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
			sudutY2=-3;//+1*sin(sudutNgurang*3.14/180);
			sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
			speed=100;//cos(sudutNgurang*3.14/360);
			//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
			inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,512,512,'2');
		}
	
	f2=0;
	//cout<<nilaisensor<<endl;
	//usleep(500000);
	sleep(100000);
	}
//	sleep(30000);
}

int main(){
	system("clear");
	if(dxl_initialize(0,1)==0){
	        cout<<"Error connecting"<<endl;
        	return 0;
	}
	else{
		cout<<"connect success!"<<endl;
	}

	//ChangeCursorStatus(FALSE);
	typedef void (*TMenuOption)();

	int ItemCount=7;
	int MenuChoice=1;
	char key;
	
	TMenuOption *MenuOption = new TMenuOption[ItemCount];
	MenuOption[0] = MenuTuning1;
	MenuOption[1] = MenuTuning2;
	MenuOption[2] = MenuTuning3;
	MenuOption[3] = MenuTuning4;
	MenuOption[4] = MenuTuning5;
	MenuOption[5] = MenuTuning6;
	MenuOption[6] = ExitOpt;
	while(1){
		status = false;
		system("clear");
		gotoxy(0,0); cout << "Console Tuning Robot Humanoid El-Pistolero";

		for(int i = 0; i < ItemCount; i++){
			gotoxy(1,3+i);
			MenuChoice==i+1 ? cout << " ->" : cout << "   ";
			cout << MenuItems()[i] << endl;
		}
		
		key = toupper(getch());

		switch(key){
		case 'X':
			try{
				status=true; 
				(*MenuOption[MenuChoice-1])();
			}catch(...){
			}
			break;
		case 'A':
			MenuChoice++;
			if(MenuChoice > ItemCount){
				MenuChoice=1;
			}
			break;
		case 'B':
			MenuChoice--;
			if(MenuChoice < 1){
				MenuChoice=ItemCount;
			}
			break;
		case '\033':
			try{
				(*MenuOption[ItemCount-1])();
			}catch(...){
			}
			break;
		default:
			if(key >= '1' && key <= char(ItemCount+'0')){
				try{
					(*MenuOption[int(key)-'0'-1])();
				}catch(...){
				}
			}
		}
	}
	delete MenuOption;


/*
	while(1){
		verse();
		usleep(3000);
	}*/
	return 0;
}


#endif
