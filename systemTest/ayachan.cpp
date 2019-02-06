#ifndef AYACHAN_C 
#define AYACHAN_C

#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"
//#include "/home/pi/systemTest/searchimage.cpp"
//#include "/home/pi/systemTest/pointImage.cpp"

//char *compro;
float nilaisensor,nilaisensor2;
double epka,epki,edka,edki,epx,edx,epy,edy,erry,outpx,outpy,outpka,outpki,outdka,outdki,outdx,outdy,outpy2,outdy2,epy2,edy2,torka ;
double kpx = 0;//.025; //pid kanan kiri
double kdx = 0;//.4;	//pid kanan kiri
double kpy = 0;//.014;	//pid kanan kiri
double kdy = 0;//.0068;	//pid kanan kiri
double kpka = 0.9;// kp tangan kanan
double kdka = 0; // kd tangan kanan
double kpki = 1; // kp tangan kiri
double kdki = 0; // kd tangan kiri
double kpy2 = 0;	//pid depan belakang
double kdy2 = 0;	//pid depan belakang
double edysblm = 0; //pid kanan kiri
double edkasblm = 0;	//pid tangan
double edkisblm = 0;	//pid tangan
double edxsblm = 0; //pid kanan kiri
double pidx = 0;
double pidy= 0;
double setpointx = 186;	//pid kanankiri dan tangan
double setpointy = 0;//1.1;
double pidka = 0;
double pidki = 0;
double setpointy2 = 270; // pid depan belakang
double pidy2 = 0;
double edy2sblm = 0;	//pid depan belakang
int a,b,c,tkan,tki;
double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
int speed;
int comser = 0;
int j = 1;
int ge;
pthread_mutex_t lock2,lock3,lock4;

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

//void *geraktangan(void *arg){
void geraktangan(){	
//comser=0;

//cout<<"aktif tangan"<<endl;

	int id1[] = {3,4};
	int data1[] = {tkan,tki};

//	pthread_mutex_lock(&lock3);
	setAllSpeed(100);
    kirimPacketGerak1(2, 1, 1, id1, data1, 1, 1000);
//	pthread_mutex_unlock(&lock3);
	
//com=1;
}

void sensen(){

	enableIMU();
	nilaisensor = bacaACC2(Pacc_raw);
	nilaisensor2 = bacaACC3(Pacc_raw);
	
}

//void *berdiri(void *arg){	
void berdiri(){	

x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=0;
        z=0.5;
        sudutX=0;
        sudutY=7;
        sudutZ=0;
        x2=-14;
        y2=0;
        z2=0;
        sudutX2=0;
        sudutY2=7;
        sudutZ2=0;
	dxl_write_word(3,30,300);
	dxl_write_word(4,30,744);
	//============
	sensen();
	
	tkan = 300;
	tki = 744;
	//===============
//	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,'2');
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,1023,1023,'2');

	sleep(3);

	   }
	   
	//    MIRING   //
void miring(){
//void *miring(void *arg){
//==================================
//if(i==0){
	edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;//}
 //============================
 
	 for(int i=0;i<=38;i++){
       	
 sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=1.1*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=-5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7;//+sin(sudutNgurang*3.14/180);
        sudutZ=0.5;//3*(pow(sin(sudutNgurang*3.14/180),2));
	x2=-14;//+sin(sudutNgurang*3.14/180)*2;
      	y2=1.1*sin(sudutNgurang*3.14/180);//2.4
      	z2=0;
        sudutX2=5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
	  speed=100;//cos(sudutNgurang*3.14/360);

	sensen();
//	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,'2');
//	usleep(1000);
	piddepanbelakang();
	pidtangan();
	geraktangan();
	usleep(200);
//cout<<"hitung"<<endl;
	/*i++;
	if(i==39){
		comger=2;
		i=0;
	}*/

}
	//  ANKAT KAKI DAN MENDARAT //
//void angkatmendarat(){
//void *angkatmendarat(void *arg){
	//====================
//if(i==0){
	edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;//}
//====================
	 for(int i=0;i<=76;i++){

 sudutNgurang=2.4*i;
	x=-14+1*sin(sudutNgurang*3.14/180)*1.4;
        y=1.1+0*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
        z=0.5+0.5-0.5*cos(sudutNgurang*3.14/180);
	sudutX=-5.5-16*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	
	sudutY=7+pidx;//+4*cos(sudutNgurang*3.14/180);
        sudutZ=0;//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-2*sin(sudutNgurang*3.14/180);
      	y2=1.1+4*sin(sudutNgurang*3.14/180);// + pidy;//sin(sudutNgurang*3.14/180);//2.4
      	z2=-0.5+0.5*cos(sudutNgurang*3.14/180);
        sudutX2=5.5-13*sin(sudutNgurang*3.14/180);// - pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7+pidx;//+4*sin(sudutNgurang*3.14/180);
    sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	speed=100;//cos(sudutNgurang*3.14/360);
	//============
	sensen();

	//============
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,'2');
	//inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	pidkanankiri();
	//usleep(1000);
	piddepanbelakang();
	pidtangan();
	geraktangan();
	//usleep(500);

/*	i++;
	if(i==77){
		comger=3;
		i=0;
		}*/
	}
		
//void balikinmiring(){
//===============
//if(i==0){
	edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;//}
//======================	
   for(int i=0;i<=38;i++){

 sudutNgurang=2.4*i;
        x=-14;//-sin(sudutNgurang*3.14/180)*2;
        y=1.1-1.1*sin(sudutNgurang*3.14/180);//2.4
        z=1.5;
        sudutX=-5.5+5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*2;
        y2=1.1-1.1*sin(sudutNgurang*3.14/180);//2.4
        z2=-1;//0.2;//.2;
        sudutX2=5.5-5.5*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2 
	speed=100;//cos(sudutNgurang*3.14/360);
	//=============
	sensen();
	
	//====================
//	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');     
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,'2');

	piddepanbelakang();
	pidtangan();
	geraktangan();
	usleep(200);
/*	i++;
	if(i==39){
		comger=4;
		i=0;*/
	}
//if(ge==0||ge==3){comger=0;}else{
comger=2;
//}

}
	// MIRING //
//for(int j=0;j<=76;j++){
void miring2(){
//================
//if(i==0){
	edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;//}
//==================	

	for(int i=0;i<=38;i++){
	sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*2;
        y=-1*sin(sudutNgurang*3.14/180);//2.4
        z=1.5;//cos(sudutNgurang*3.14/180);
        sudutX=4*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7;//+1*sin(sudutNgurang*3.14/180);
        sudutZ=0;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=-1*sin(sudutNgurang*3.14/180);//2.4
        z2=-1;//0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=-4*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7;//+1*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
         speed=100;//cos(sudutNgurang*3.14/360);
	//================
	sensen();
	
	//================
//	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');      
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,'2');
	piddepanbelakang();
	pidtangan();
	geraktangan();
	
	usleep(200);
	
	/*i++;
	if(i==39){
		comger=5;
		i=0;
	}*/
}
        //usleep(100);
	//ANGKAT KAKI DAN MENDARAT/
//void angkatmendarat2(){	
	//======================
//if(i==0){
	pidy2 = 0;
	edy2sblm = 0;
	edysblm = 0;
	edxsblm = 0;
	pidx = 0;
	pidy= 0;
	edkasblm = 0;
	edkisblm = 0;//}
	//==================	
	for(int i=0;i<=76;i++){
  sudutNgurang=2.4*i;
        x=-14+0*sin(sudutNgurang*3.14/180);
	y=-1-1*sin(sudutNgurang*3.14/180);//- pidy;//2.4
        z=1.5*cos(sudutNgurang*3.14/180);
        sudutX=4-18*sin(sudutNgurang*3.14/180);//- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7+pidx;//3*sin(sudutNgurang*3.14/180);//+sin(sudutNgurang*3.14/180);
	// speed=200*cos(sudutNgurang*3.14*0.5/180);
        sudutZ=0;//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14+1.5*sin(sudutNgurang*3.14/180)*1.5;
        y2=-1-3*sin(sudutNgurang*3.14/180);//2.4
        z2=-1*cos(sudutNgurang*3.14/180);
        sudutX2=-4-6*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7+pidx;//3*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	 speed=100;//cos(sudutNgurang*3.14/360);
	//===============
	sensen();
	
	//===========
//	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,'2');
	//usleep(2000);
	
	pidkanankiri();
	piddepanbelakang();
	pidtangan();
	geraktangan();

//	}
/*	i++;
	if(i==77){
		comger=6;
		i=0;
	}*/
}
				
	// BALIKIN MIRING//

//void balikinmiring2(){
//=======================
//if(i==0){
	pidy2 = 0;
	edy2sblm = 0;
	edkasblm = 0;
	edkisblm = 0;//}
//==========================
	
	for(int i=0;i<=38;i++){
          sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*1.4;
        y=-1+1*sin(sudutNgurang*3.14/180);//2.4
        z=-1.5;//cos(sudutNgurang*3.14/180);
	sudutX=4-4*sin(sudutNgurang*3.14/180);
        sudutY=7;//-1*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=-1+1*sin(sudutNgurang*3.14/180);//2.4
        z2=1;//-0.2;//cos(sudutNgurang*3.14/180);
	sudutX2=-4+4*sin(sudutNgurang*3.14/180);
        sudutY2=7;//-1*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
         speed=100;//cos(sudutNgurang*3.14/360);
//=============
	sensen();
	
//=====================	
//	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,'2');
	piddepanbelakang();
	pidtangan();
	geraktangan();	
	usleep(200);
	
/*	i++;
	if(i==39){
		comger=7;
		i=0;*/
	}
//if(ge==0||ge==3){comger=0;}else{
comger=3;
//}
}

	// MIRING //
 void miring3(){
//===========
//if(i==0){
	pidy2 = 0;
	edy2sblm = 0;
	edkasblm = 0;
	edkisblm = 0;//}
//=============
	
	for(int i=0;i<=38;i++){
  sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*2;
        y=2*sin(sudutNgurang*3.14/180);//2.4
        z=-1.5;//cos(sudutNgurang*3.14/180);
        sudutX=-4*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=2*sin(sudutNgurang*3.14/180);//2.4
        z2=1;//-0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=4*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//cos(sudutNgurang*3.14/360);
	//==============
	sensen();
	
	//=====================
//    inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,'2');
	piddepanbelakang();
	pidtangan();
	geraktangan();
	usleep(200);			
	
/*	i++;
	if(i==39){
		comger=8;
		i=0;
		}*/
}

        //ANGKAT KAKI DAN MENDARAT//
//void angkatmendarat3(){	
	//=======================
//if(i==0){
	edysblm = 0;
	edxsblm = 0;
	pidx = 0;
	pidy= 0;
    edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;//}
	//=======================
		
		for(int i=0;i<=76;i++){
  sudutNgurang=2.4*i;
        x=-14+1.2*sin(sudutNgurang*3.14/180);
        y=2;//+0.5*sin(sudutNgurang*3.14/180);//2.4
        z=-1.5*cos(sudutNgurang*3.14/180);
        sudutX=-4-16*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=7;//+3*sin(sudutNgurang*3.14/180);
        sudutZ=0;//cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*1.5;
        y2=2+1*sin(sudutNgurang*3.14/180);//+pidy;//2.4
        z2=1*cos(sudutNgurang*3.14/180);
        sudutX2=4-17*sin(sudutNgurang*3.14/180);//- pidx;//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=7;//+3*sin(sudutNgurang*3.14/180);//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=150;//*cos(sudutNgurang*3.14/360);
	//==================
	sensen();
	
	//================
//    inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,'2');
	//usleep(2000);
	
	pidkanankiri();
	piddepanbelakang();
	pidtangan();
	geraktangan();	
	
/*	i++;
	if(i==77){
		comger=9;
		i=0;
		}*/
	
}

        // BALIKIN MIRING//
//void balikinmiring3(){
		//============	
//if(i==0){
	pidy2 = 0;
	edy2sblm = 0;
	edkasblm = 0;
	edkisblm = 0;//}
  //=================  
    
	for(int i=0;i<=38;i++){
    sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*1.4;
        y=2-2*sin(sudutNgurang*3.14/180);//2.4
        z=1.5;//cos(sudutNgurang*3.14/180);
        sudutX=-4+4*sin(sudutNgurang*3.14/180);
        sudutY=7;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//-2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=2-2*sin(sudutNgurang*3.14/180);//2.4
        z2=-1;//0.2;//cos(sudutNgurang*3.14/180);
        sudutX=4-4*sin(sudutNgurang*3.14/180);
        sudutY2=7;//-sin(sudutNgurang*3.14/180);
        sudutZ2=0;//-3*(pow(sin(sudutNgurang*3.14/180),2));
        speed=100;//*cos(sudutNgurang*3.14/360);
	//=================
	sensen();
	
	//====================
//    inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,'2');
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,1023,1023,'2');
	piddepanbelakang();
	pidtangan();
	geraktangan();
	usleep(200);
	
/*	i++;
	if(i==39){
		comger=4;
		i=0;
		j++;*/
	}
/*if(ge==0||ge==3){comger=0;}
else if(ge==2){
	//j++;
	comger=2;}else
	{comger=2;}	*/	

comger=2;
}

//=============================================            kkputar(kiri)        =====================================================

 void kiriputar1(){
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
            torka = -torka;} 
		else if (torka > 0){
            torka = torka;}
		else if (torka>1023){
            torka=1023;
            }
        if (epx < 0){
            epy = -epy;//cos(epx*0.017);
			}
		else if (epx > 0){
            epy = epy;//cos(epx*0.017);
            }
	y=y+0.05*epy;
	y2=y2+0.05*epy;	
	inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,speed,torka,torka,'2');
	usleep(200);
	piddepanbelakang();
	pidtangan();
	geraktangan();
	}
		
	//  ANKAT KAKI DAN MENDARAT //

	//====================
	edysblm = 0;
	edxsblm = 0;
	pidx = 0;
	pidy= 0;
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
	
	piddepanbelakang();
	pidtangan();
	geraktangan();
	pidkanankiri();
	}

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
	usleep(200);
	piddepanbelakang();
	pidtangan();
	geraktangan();
}
comkir=2;
}
void kiriputar2(){
	// MIRING 2//	

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
	usleep(200);
	piddepanbelakang();
	pidtangan();
	geraktangan();
	}

	//ANGKAT KAKI DAN MENDARAT2/
	
	//======================
	edysblm = 0;
	edxsblm = 0;
	pidx = 0;
	pidy= 0;
    edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;
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
	piddepanbelakang();
	pidtangan();
	geraktangan();
	pidkanankiri();
	}

	// BALIKIN MIRING2//
//=======================
	pidy2 = 0;
	edy2sblm = 0;
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
	usleep(200);
	piddepanbelakang();
	pidtangan();
	geraktangan();
	}
	comkir=1;
 }	

//===============================================       putarss(kanan)        ==============================================================
void kananputar1(){
// MIRING //

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
        y=3.1*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=10+pidy2;//+sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
	x2=-14;//+sin(sudutNgurang*3.14/180)*2;
      	y2=3.1*sin(sudutNgurang*3.14/180);//2.4
      	z2=0;
        sudutX2=2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
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
	usleep(200);
	piddepanbelakang();
	pidtangan();
	geraktangan();
	}
	
	//  ANKAT KAKI DAN MENDARAT //

	//====================
	edysblm = 0;
	edxsblm = 0;
	pidx = 0;
	pidy= 0;
    edkasblm = 0;
	edkisblm = 0;
	pidy2 = 0;
	edy2sblm = 0;
//====================

	 for(int i=0;i<=76;i++){

	
	 sudutNgurang=2.4*i;
	x=-14+2*sin(sudutNgurang*3.14/180)*1.4;
        y=3.1+0*sin(sudutNgurang*3.14/180);//cos(sudutNgurang*3.14/360);//2.4
        z=0;//-0.5+0.5*cos(sudutNgurang*3.14/180);
	sudutX=-2-24*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/	
	sudutY=10+3*cos(sudutNgurang*3.14/180);//+pidy2;
        sudutZ=-3+3*cos(sudutNgurang*3.14/180);//-0.5+0.5*cos(sudutNgurang*3.14/180);//1*sin(sudutNgurang*3.14/180)*1.4;;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-0*sin(sudutNgurang*3.14/180);
      	y2=3.1+0*sin(sudutNgurang*3.14/180) ;//+ pidy;//sin(sudutNgurang*3.14/180);//2.4
      	z2=0;//0.5-0.5*cos(sudutNgurang*3.14/180);
        sudutX2=2-10*sin(sudutNgurang*3.14/180) - pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10+3*sin(sudutNgurang*3.14/360);//+pidy2;
    sudutZ2=-3+3*cos(sudutNgurang*3.14/180);//-1+1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
  	speed=200;//cos(sudutNgurang*3.14/360);
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
	piddepanbelakang();
	pidtangan();
	geraktangan();
	pidkanankiri();
	}
	
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
        y=3.1-3.1*sin(sudutNgurang*3.14/180);//2.4
        z=0;
        sudutX=-2+2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=10+pidy2;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=-6;//-2;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//+sin(sudutNgurang*3.14/180)*2;
        y2=3.1-3.1*sin(sudutNgurang*3.14/180);//2.4
        z2=0;//0.2;//.2;
        sudutX2=2-2*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10+pidy2;//+2*sin(sudutNgurang*3.14/180);
        sudutZ2=-6;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2 
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
	usleep(200);
	piddepanbelakang();
	pidtangan();
	geraktangan();
	}
	comkan=2;
}	

void kananputar2(){
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
        y=-2*sin(sudutNgurang*3.14/180);//2.4
        z=0;//cos(sudutNgurang*3.14/180);
        sudutX=3*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=10-0*sin(sudutNgurang*3.14/180)+pidy2;

        sudutZ=-6;//2;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*2.4;
        y2=-2*sin(sudutNgurang*3.14/180);//2.4
        z2=0;//0.2;//cos(sudutNgurang*3.14/180);
        sudutX2=-3*sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10-0*sin(sudutNgurang*3.14/180)+pidy2;
        sudutZ2=-6;//-2;//-3*(pow(sin(sudutNgurang*3.14/180),2));
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
	usleep(200);
	
	piddepanbelakang();
	pidtangan();
	geraktangan();
		}

	//ANGKAT KAKI DAN MENDARAT/
	
	//======================
	pidy2 = 0;
	edy2sblm = 0;
	edysblm = 0;
	edxsblm = 0;
	pidx = 0;
	pidy= 0;
	edkasblm = 0;
	edkisblm = 0;
	//==================
	
	for(int i=0;i<=76;i++){
 	 sudutNgurang=2.4*i;
        x=-14;//+2*sin(sudutNgurang*3.14/180);
	y=-2-2*sin(sudutNgurang*3.14/180)- pidy;//2.4
        z=0*cos(sudutNgurang*3.14/360);
        sudutX=3-10*sin(sudutNgurang*3.14/180)- pidx;/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY=5+0*sin(sudutNgurang*3.14/180)+pidy2;//+sin(sudutNgurang*3.14/180);
	// speed=200*cos(sudutNgurang*3.14*0.5/180);
        sudutZ=-6+3-3*cos(sudutNgurang*3.14/180);//8*sin(sudutNgurang*3.14/360);//-0.5-0.5*cos(sudutNgurang*3.14/180);//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14+2*sin(sudutNgurang*3.14/180);
        y2=-2-2*sin(sudutNgurang*3.14/180);//2.4
        z2=0*cos(sudutNgurang*3.14/180);
        sudutX2=-3-24*sin(sudutNgurang*3.14/180);//sin(sudutNgurang*3.14/180);/*(pow(sin(sudutNgurang*3.14/180),2));*/
        sudutY2=10+0*sin(sudutNgurang*3.14/180)+pidy2;
        sudutZ2=-6+3-3*cos(sudutNgurang*3.14/180);//8*sin(sudutNgurang*3.14/360);//-1-1*cos(sudutNgurang*3.14/180);//-3*(pow(sin(sudutNgurang*3.14/180),2));
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

	piddepanbelakang();
	pidtangan();
	geraktangan();
	pidkanankiri();
	}

	// BALIKIN MIRING//
//=======================
		pidy2 = 0;
	edy2sblm = 0;
	edkasblm = 0;
	edkisblm = 0;
//==========================
	
	for(int i=0;i<=38;i++){
         sudutNgurang=2.4*i;
        x=-14;//+sin(sudutNgurang*3.14/180)*1.4;
        y=-2+2*sin(sudutNgurang*3.14/180);//2.4
        z=0;//cos(sudutNgurang*3.14/180);
	sudutX=3-3*sin(sudutNgurang*3.14/180);
        sudutY=10+pidy2;//+2*sin(sudutNgurang*3.14/180);
        sudutZ=0;//3*(pow(sin(sudutNgurang*3.14/180),2));
        x2=-14;//-sin(sudutNgurang*3.14/180)*1.4;
        y2=-2+2*sin(sudutNgurang*3.14/180);//2.4
        z2=0;//-0.2;//cos(sudutNgurang*3.14/180);
	sudutX2=-3+3*sin(sudutNgurang*3.14/180);
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
	usleep(200);
	piddepanbelakang();
	pidtangan();
	geraktangan();
		}
	comkan=1;
}
#endif
