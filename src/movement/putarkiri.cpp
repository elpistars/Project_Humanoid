
#ifndef PUTARKIRI_CPP
#define PUTARKIRI_CPP

#include <iostream>
#include <unistd.h>
#include "movement/foot_ik.h"

using namespace std;

void putarkiri(){

   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();     

   double f2=0;
   int x_head=512;
   int y_head=512;
   
   f2=0;
   Invers.setParam(90,0.05,100);        //sudut,inc_sudut,speed
   while(f2<=Invers.param.sudut){
        
        f2+=Invers.param.inc_sudut;
        Kanan.setParam(-12.4,   //x
         -2*sin(f2*3.14/180),   //y
         0.3,                   //z
         2*sin(f2*3.14/180),    //sudutX
         22,                    //sudutY
         0                      //sudutZ
        );

        Kiri.setParam(-12,      //x
         -2*sin(f2*3.14/180),   //y
         0,                     //z
         2*sin(f2*3.14/180),    //sudutX
         22,                    //sudutY
         0                      //sudutZ
        );

        //uncommnent code dibawah buat print param kaki
        //kalau mau print nilai akhir pindahin kodenya ke abis '}'
        /* Kanan.printParam((char*)"Kanan :");
	Kiri.printParam((char*)"Kiri :"); */

        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512,480,520,270,750,250,750,x_head,y_head);
   }
   cout<<endl<<endl;
   //update nilai temp_joint
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
   //sleep(5000);
   usleep(1000);
// ------------------  ANKAT KAKI DAN MENDARAT //
   f2=0;
   Invers.setParam(180,0.2,200);
   while(f2<=Invers.param.sudut){
        f2+=Invers.param.inc_sudut;
        Kanan.setParam(temp_joint[0].x,
        temp_joint[0].y,
        temp_joint[0].z,
        temp_joint[0].sudutX - 4*sin(f2*3.14/180),
        temp_joint[0].sudutY,
        temp_joint[0].sudutZ + 512 - 50*sin(f2*3.14/180)
        );
        
        Kiri.setParam(temp_joint[1].x + 1.4*sin(f2*3.14/180),		
	temp_joint[1].y,					
	temp_joint[1].z,										
	temp_joint[1].sudutX - 4*sin(f2*3.14/180),				
	temp_joint[1].sudutY,									
	temp_joint[1].sudutZ + 512 - 50*sin(f2*3.14/180) 				
	);

        //uncommnent code dibawah buat print param kaki
        //kalau mau print nilai akhir pindahin kodenya ke abis '}'
        /* Kanan.printParam((char*)"Kanan :");
	Kiri.printParam((char*)"Kiri :"); */

        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512);  
   }
   cout<<endl<<endl;
   //update nilai temp_joint
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();

   usleep(5000);
   //usleep(1000);
// --------------------------- BALIKIN MIRING //
   f2=0;
   Invers.setParam(90,0.2,250);
   while(f2<=Invers.param.sudut){
        f2+=Invers.param.inc_sudut;
        Kanan.setParam(temp_joint[0].x,
        temp_joint[0].y + 2*sin(f2*3.14/180),
        temp_joint[0].z,
        temp_joint[0].sudutX + 2*sin(f2*3.14/180),
        temp_joint[0].sudutY,
        temp_joint[0].sudutZ + 50*sin(f2*3.14/180)
        );
        
        Kiri.setParam(temp_joint[1].x - 1.4*sin(f2*3.14/180),		
	temp_joint[1].y + 2*sin(f2*3.14/180),					
	temp_joint[1].z,										
	temp_joint[1].sudutX + 2*sin(f2*3.14/180),				
	temp_joint[1].sudutY,									
	temp_joint[1].sudutZ 				
	);	

        //uncommnent code dibawah buat print param kaki
        //kalau mau print nilai akhir pindahin kodenya ke abis '}'
        /* Kanan.printParam((char*)"Kanan :");
	Kiri.printParam((char*)"Kiri :"); */

        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512);
   }
   cout<<endl<<endl;
   //update nilai temp_joint
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
   usleep(700000); //800000
        //sleep(1000);

// ----------------------------- MIRING //
   f2=0;
   Invers.setParam(90,0.01,100);
   while(f2<=Invers.param.sudut){
        f2+=Invers.param.inc_sudut;
        Kanan.setParam(temp_joint[0].x,
        temp_joint[0].y + 2*sin(f2*3.14/180),
        temp_joint[0].z,
        temp_joint[0].sudutX - 2*sin(f2*3.14/180),
        temp_joint[0].sudutY,
        temp_joint[0].sudutZ
        );
        
        Kiri.setParam(temp_joint[1].x,		
	temp_joint[1].y + 2*sin(f2*3.14/180),					
	temp_joint[1].z,										
	temp_joint[1].sudutX + 2*sin(f2*3.14/180),				
	temp_joint[1].sudutY,									
	temp_joint[1].sudutZ 				
	);

        //uncommnent code dibawah buat print param kaki
        //kalau mau print nilai akhir pindahin kodenya ke abis '}'
        /* Kanan.printParam((char*)"Kanan :");
	Kiri.printParam((char*)"Kiri :"); */

        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512);
   }
   cout<<endl<<endl;
   //update nilai temp_joint
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
   //sleep(5000);
   usleep(5000);
// ------------------------ ANGKAT KAKI DAN MENDARAT/
   f2=0;
   Invers.setParam(180,0.1,250);
   while(f2<=Invers.param.sudut){
        f2+=Invers.param.inc_sudut;
        Kanan.setParam(temp_joint[0].x,
        temp_joint[0].y + 2*sin(f2*3.14/180),
        temp_joint[0].z,
        temp_joint[0].sudutX - 2*sin(f2*3.14/180),
        temp_joint[0].sudutY,
        temp_joint[0].sudutZ
        );
        
        Kiri.setParam(temp_joint[1].x,		
	temp_joint[1].y + 2*sin(f2*3.14/180),					
	temp_joint[1].z,										
	temp_joint[1].sudutX + 2*sin(f2*3.14/180),				
	temp_joint[1].sudutY,									
	temp_joint[1].sudutZ 				
	);

        //uncommnent code dibawah buat print param kaki
        //kalau mau print nilai akhir pindahin kodenya ke abis '}'
        /* Kanan.printParam((char*)"Kanan :");
	Kiri.printParam((char*)"Kiri :"); */

        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512);
   }
   cout<<endl<<endl;
   //update nilai temp_joint
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
   usleep(5000);
//---------------------- BALIKIN MIRING --------------------------------//
   f2=0;
   Invers.setParam(90,0.1,250);
   while(f2<=90){
        f2+=Invers.param.inc_sudut;
        Kanan.setParam(temp_joint[0].x,
        temp_joint[0].y + 2*sin(f2*3.14/180),
        temp_joint[0].z,
        temp_joint[0].sudutX - 2*sin(f2*3.14/180),
        temp_joint[0].sudutY,
        temp_joint[0].sudutZ
        );
        
        Kiri.setParam(temp_joint[1].x,		
	temp_joint[1].y + 2*sin(f2*3.14/180),					
	temp_joint[1].z,										
	temp_joint[1].sudutX + 2*sin(f2*3.14/180),				
	temp_joint[1].sudutY,									
	temp_joint[1].sudutZ 				
	);

        //uncommnent code dibawah buat print param kaki
        //kalau mau print nilai akhir pindahin kodenya ke abis '}'
        /* Kanan.printParam((char*)"Kanan :");
	Kiri.printParam((char*)"Kiri :"); */

        Invers.inverseAndre(Kanan.getParam(),Kiri.getParam(),Invers.param.speed,512,512);
   }
   cout<<endl<<endl;
   //update nilai temp_joint
   temp_joint[0]=Kanan.getParam();
   temp_joint[1]=Kiri.getParam();
}

#endif


