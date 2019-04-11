
#ifndef PUTARKANAN_CPP
#define PUTARKANAN_CPP

#include <iostream>
#include <unistd.h>
#include "movement/foot_ik.h"

using namespace std;


void putarkanan(){ 	
	//simpan nilai parameter kanan di joint[0] kiri di joint[1]
	temp_joint[0]=Kanan.getParam();
	temp_joint[1]=Kiri.getParam();
	
	Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),100,512,512,480,520,270,750,250,750,x_head,y_head);
        //cin>>speed;
    sleep(2);//5

	// MIRING kiri //
	f2=0;
	Invers.setParam(90,0.1,200); // max_sudut, inc_sudut, speed
	while(f2<=Invers.param.sudut){
		f2+=Invers.param.inc_sudut;
        
		Kanan.setParam(temp_joint[0].x, 			//x -12.4
		 temp_joint[0].y - 2, //*sin(f2*3.14/180),  	//y 2
		 temp_joint[0].z,				  			//z 0.3
		 temp_joint[0].sudutX - 2 *sin(f2*3.14/180), //sudutX -2
		 temp_joint[0].sudutY + 2,				  		//sudutY 22
		 temp_joint[0].sudutZ						//sudutZ 0
		);				  

		Kiri.setParam(temp_joint[1].x, 				// -12
		temp_joint[1].y - 2,//*sin(f2*3.14/180), 		// 2
		temp_joint[1].z, 							// 0
		temp_joint[1].sudutX -2 *sin(f2*3.14/180), 	// -2
		temp_joint[1].sudutY + 2, 						// 22
		temp_joint[1].sudutZ 						// 0
		);
		Kanan.printParam((char*)"Kanan :");
		Kiri.printParam((char*)"Kiri :");
		Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512,480,520,270,750,250,750,x_head,y_head);
	}
	cout<<endl<<endl;
	//update nilai temp_joint
	temp_joint[0]=Kanan.getParam();
	temp_joint[1]=Kiri.getParam();

	f2=0;
	sleep(1000);
// ------------------  ANKAT KAKI DAN MENDARAT //
	Invers.setParam(90,0.08,200);
	while(f2<=Invers.param.sudut){
		f2+=Invers.param.inc_sudut;
		
		Kanan.setParam(temp_joint[0].x+5*sin(f2*3.14/180),		// -7.4
		temp_joint[0].y - 1* sin(f2*3.14/180),					// 1
		temp_joint[0].z,										// 0.3
		temp_joint[0].sudutX + 1*sin(f2*3.14/180),				// -1
		temp_joint[0].sudutY,									// 22
		temp_joint[0].sudutZ + 20*sin(f2*3.14/180)				// 20
		);

		Kiri.setParam(temp_joint[1].x + 5*sin(f2*3.14/180),		// -7
		temp_joint[1].y - 1*sin(f2*3.14/180),					// 1
		temp_joint[1].z,											// 0
		temp_joint[1].sudutX - 3*sin(f2*3.14/180),				// -5
		temp_joint[1].sudutY,									// 22
		temp_joint[1].sudutZ + 15*sin(f2*3.14/180)				// 15
		);
		Kanan.printParam((char*)"Kanan :");
		Kiri.printParam((char*)"Kiri :");

		Invers.inverseAndre(Kanan.getParam(), Kiri.getParam(),Invers.param.speed,512,512,480,520,270,750,250,750,x_head,y_head);

	}

	cout<<endl<<endl;
	//update nilai temp_joint
	temp_joint[0]=Kanan.getParam();
	temp_joint[1]=Kiri.getParam();

	f2=0;
	usleep(5000);
// --------------------------- BALIKIN MIRING //
	Invers.setParam(90,0.2,200);
	while(f2<=Invers.param.sudut){
		f2+=Invers.param.inc_sudut;

		Kanan.setParam(temp_joint[0].x-5*sin(f2*3.14/180),		// -12.4
		temp_joint[0].y*(1-sin(f2*3.14/180)),					// 0
		temp_joint[0].z,										// 0.3
		temp_joint[0].sudutX*(1-sin(f2*3.14/180)),				// 0
		temp_joint[0].sudutY,									// 22
		temp_joint[0].sudutZ									// 20
		);

		Kiri.setParam(temp_joint[1].x - 5*sin(f2*3.14/180),		// -12
		temp_joint[1].y*(1-sin(f2*3.14/180)),					// 0
		temp_joint[1].z,										// 0
		temp_joint[1].sudutX*(1-sin(f2*3.14/180)),				// 0
		temp_joint[1].sudutY,									// 22
		temp_joint[1].sudutZ 									// 15
		);

		Kanan.printParam((char*)"Kanan :");
		Kiri.printParam((char*)"Kiri :");

		Invers.inverseAndre(Kanan.getParam(), Kiri.getParam(),Invers.param.speed,512,512,480,520,270,750,250,750,x_head,y_head);
	}

	cout<<endl<<endl;

	//update nilai temp_joint
	temp_joint[0]=Kanan.getParam();
	temp_joint[1]=Kiri.getParam();

	
// ----------------------------- MIRING //	
	sleep(5000);
	f2=0;
	Invers.setParam(90,0.2,200);
	while(f2<=Invers.param.sudut){
		f2+=Invers.param.inc_sudut;
		Kanan.setParam(temp_joint[0].x,							// -12.4
		temp_joint[0].y - 2*sin(f2*3.14/180),					// -2
		temp_joint[0].z,										// 0.3
		temp_joint[0].sudutX + 2*sin(f2*3.14/180),				// 2
		temp_joint[0].sudutY,									// 22
		temp_joint[0].sudutZ									// 20
		);

		Kiri.setParam(temp_joint[1].x,							// -12
		temp_joint[1].y - 2*sin(f2*3.14/180),					// -2
		temp_joint[1].z,											// 0
		temp_joint[1].sudutX + 2*sin(f2*3.14/180),				// 2
		temp_joint[1].sudutY,									// 0
		temp_joint[1].sudutZ 									// 15
		);

		Kanan.printParam((char*)"Kanan :");
		Kiri.printParam((char*)"Kiri :");

		Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512,480,520,270,750,250,750,x_head,y_head);

	}
	usleep(5000);

	cout<<endl<<endl;

	//update nilai temp_joint
	temp_joint[0]=Kanan.getParam();
	temp_joint[1]=Kiri.getParam();

// ------------------------ ANGKAT KAKI DAN MENDARAT/
	f2=0;
	Invers.setParam(90,0.2,200);
	while(f2<=Invers.param.sudut){
		f2=f2+Invers.param.inc_sudut;
		
		Kanan.setParam(temp_joint[0].x,							// -12.4
		temp_joint[0].y + 0.5*sin(f2*3.14/180),					// -1.5
		temp_joint[0].z,										// 0.3
		temp_joint[0].sudutX + 10*sin(f2*3.14/180),				// 12
		temp_joint[0].sudutY,									// 22
		temp_joint[0].sudutZ									// 20
		);

		Kiri.setParam(temp_joint[1].x + 8*sin(f2*3.14/180),		// -2
		temp_joint[1].y + 0.5*sin(f2*3.14/180),					// -1.5
		temp_joint[1].z,										// 0
		temp_joint[1].sudutX + 30*sin(f2*3.14/180),				// 32
		temp_joint[1].sudutY,									// 0
		temp_joint[1].sudutZ + 5*sin(f2*3.14/180) 				// 15
		);

		Kanan.printParam((char*)"Kanan :");
		Kiri.printParam((char*)"Kiri :");

		Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512,480,520,270,750,250,750,x_head,y_head);
	}
	usleep(5000);

	cout<<endl<<endl;

	//update nilai temp_joint
	temp_joint[0]=Kanan.getParam();
	temp_joint[1]=Kiri.getParam();
	
	// BALIKIN MIRING//
	f2=0;
	Invers.setParam(90,0.2,200);
	while(f2<=Invers.param.sudut){
		f2=f2+Invers.param.inc_sudut;
		
		Kanan.setParam(temp_joint[0].x,							// -12.4
		temp_joint[0].y + 1*sin(f2*3.14/180),					// -0.5
		temp_joint[0].z,										// 0.3
		temp_joint[0].sudutX + 12*sin(f2*3.14/180),				// 0
		temp_joint[0].sudutY,									// 22
		temp_joint[0].sudutZ									// 20
		);

		Kiri.setParam(temp_joint[1].x - 10*sin(f2*3.14/180),	// -12
		temp_joint[1].y + 1*sin(f2*3.14/180),					// -0.5
		temp_joint[1].z,										// 0
		temp_joint[1].sudutX - 30*sin(f2*3.14/180),				// 32
		temp_joint[1].sudutY,									// 0
		temp_joint[1].sudutZ					 				// 15
		);

		Kanan.printParam((char*)"Kanan :");
		Kiri.printParam((char*)"Kiri :");

		Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512,480,520,270,750,250,750,x_head,y_head);
	}
	usleep(500);

	//update nilai temp_joint
	temp_joint[0]=Kanan.getParam();
	temp_joint[1]=Kiri.getParam();
}

#endif
