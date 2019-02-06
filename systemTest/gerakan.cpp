

#ifndef GERAKAN_C
#define GERAKAN_C

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "library/allLib.h"
#include "library/dynamixel/syncWrite.cpp"
#include "library/movement/move.h"
#include "library/sensor/sensor.c"

using namespace cv;

int CamCommand;


void kasus(int command){
		CamCommand=command;
		setAllTorque(1023);
	switch(CamCommand){
		/*case 301:
			search(500000,1);
			sleep(1);
			break;

		case 302: 
			cout<<"gerakan 302"<<endl;
			setAllSpeed(140);
			set_parametergeser_kiri(-70,-70,-7,-7,-10,-10,9,9,0,0,0,0);
            		usleep(50000);//70000
			siapJalan(1,0);
			geser_kiri(10000); //fixedPID();
			usleep(50000);//70000
			siapJalan(1,0);
			set_parametergeser_kiri(70,70,7,7,10,10,-9,-9,0,0,0,0);
            		usleep(500000);//1000000

			set_parameterputar_kanan(-170,-170,0,0,-7,-7,10,10,0,0,0,0);
            		putar_kanan(30000);
			//fixedPID();
            		//enable_move = 0;
			siapJalan(1,0);
                	set_parameterputar_kanan(170,170,0,0,7,7,-10,-10,0,0,0,0);
            		usleep(500000);//1detik
			break;

		case 401 :
        		cout<<"Gerakan 401"<<endl;
			setAllSpeed(140);
			set_parametergeser_kanan(-70,-70,-7,-7,-10,10,9,9,0,0,0,0);
			usleep(50000);//70000
                	siapJalan(1,0);
			geser_kanan(30000);
			usleep(50000);//50000
             		siapJalan(1,0);
			set_parametergeser_kanan(70,70,7,7,10,-10,-9,-9,0,0,0,0);
            		usleep(700000);//1000000

			set_parameterputar_kiri(-170,-170,0,0,-7,-7,10,10,0,0,0,0);
			putar_kiri(30000);
			siapJalan(1,0);
			set_parameterputar_kiri(170,170,0,0,7,7,-10,-10,0,0,0,0);
            		usleep(700000);//100000
            		break;
	    
		
		case 202:
			cout<<"Gerakan 202"<<endl;
			setAllSpeed(140);
			set_parameterputar_kanan(-150,-150,0,0,-7,-7,10,10,0,0,0,0);
                	//setallspeed(180);
                	siapJalan(1,0);
               		usleep(700000);//1000000
            		putar_kanan(42000);
			//fixedPID();
			siapJalan(1,0);
                	set_parameterputar_kanan(150,150,0,0,7,7,-10,-10,0,0,0,0);

			setAllSpeed(140);
			set_parameterputar_kanan(-150,-150,0,0,-7,-7,10,10,0,0,0,0);
                	//setallspeed(180);
                	siapJalan(1,0);
               		usleep(700000);//1000000
            		putar_kanan(30000);
			//fixedPID();
			siapJalan(1,0);
                	set_parameterputar_kanan(150,150,0,0,7,7,-10,-10,0,0,0,0);

            		usleep(700000);//1000000
    			break;*/

		case 1:
			setAllTorque(950);
			cout<<"gerakan 1"<<endl;
			setAllSpeed(130);
			//set_parameterputar_kiri(0,0,0,0,20,20,30,30,0,0,0,0);
	    		//setallspeed(180);
                        set_siap_jalan(0,0,0,0,-3,-3,0,0,0,0,0,0);
			pid(285);
			siapJalan(1,pd);
			//set_siap_jalan(0,0,0,0,8,8,0,0,0,0,0,0);
                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			//usleep(100000);//1000000
			putar_kiri(30000);
			//fixedPID();
                        set_siap_jalan(0,0,0,0,-3,-3,0,0,0,0,0,0);
			//usleep(100000);
			pid(285);
			siapJalan(1,pd);
                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			//set_parameterputar_kiri(0,0,0,0,-20,-20,-30,-30,0,0,0,0);
        		//ELP_SyncHead(serX,serY,1);
            		//usleep(400000);//1000000 
   			break;

		case 2:	
			setAllTorque(950);
			cout<<"gerakan 2"<<endl;
			setAllSpeed(120); 
			//set_parameterputar_kanan(0,0,0,0,20,20,30,30,0,0,0,0);
                	//setallspeed(180);
                        set_siap_jalan(0,0,0,0,-3,-3,0,0,0,0,0,0);
                	pid(285);
			siapJalan(1,pd);
                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
               		//usleep(100000);//1000000
            		putar_kanan(30000);
			//fixedPID();
			//usleep(500000);
                        set_siap_jalan(0,0,0,0,-3,-3,0,0,0,0,0,0);
			pid(285);
			siapJalan(1,pd);
                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
                	//set_parameterputar_kanan(0,0,0,0,-20,-20,-30,-30,0,0,0,0);
        		//ELP_SyncHead(serX,serY,1);
            		//usleep(400000);//1000000 
			break;


		case 3:
			setAllTorque(950);
			cout<<"gerakan 3"<<endl;
			setAllSpeed(280);
			set_parametergeser_kiri(-60,-60,-7,-7,-10,-10,9,9,0,0,0,0);
            		usleep(70000);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			pid(283);
			siapJalan(1,pd);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			geser_kiri(22000);
			//fixedPID();
			usleep(70000);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			pid(280);
			siapJalan(1,pd);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			set_parametergeser_kiri(60,60,7,7,10,10,-9,-9,0,0,0,0);
            		usleep(700000);//1000000
            		break;

		case 4 :       
			setAllTorque(950);
			cout<<"gerakan 4"<<endl;
			setAllSpeed(140);
			set_parametergeser_kanan(-60,-60,-7,-7,-10,10,9,9,0,0,0,0);
			usleep(70000);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
                	pid(280);
			//siapJalan(1,pd);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			geser_kanan(22000);
			//fixedPID();
			usleep(70000);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
             		pid(280);
			//siapJalan(1,pd);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			set_parametergeser_kanan(60,60,7,7,10,-10,-9,-9,0,0,0,0);
            		usleep(700000);//1000000
            		break;
		
		case 5 :
			setAllTorque(850);
			cout<<"gerakan 5"<<endl;
			//ELP_SyncHead(512,512,1);
        		sleep(0.5);
        		pid(175);
			siapJalan(1,pd);
			/* work 3/21/2015
			set_siap_jalan(-70,-70,-7,-7,-10,-10,0,0,0,0,0,0);
			set_parameterjalan(-20,-20,-10,-10,-0,-0,0,0,0,0,0,0);
			*/
			//set_siap_jalan(-70,-70,-7,-7,-15,-15,0,0,0,0,0,0);
                        //set_parameterjalan(-10,-10,-12,-12,-10,-10,0,0,0,0,0,0);
			//set_parameterjalan(15,15,-12,-12,-10,-10,0,0,0,0,0,0);
			set_parameterjalan(-25,-25,-20,-20,-7,-7,0,0,0,0,0,0);
			//set_siap_jalan(0,0,0,0,-3,-3,0,0,0,0,0,0);
			//setAllSpeed(190);
			//siapJalan();
        		//ELP_SyncHead(serX,serY,1);
			pid(175);
			siapJalan(1,pd);
			usleep(1000000);
			jalan2(190,50000);
			//jalan2(190,50000);
			//usleep(700000);
			pid(175);
			siapJalan(1,pd);
			//set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			//jalan1(170,10000); //fase start
        		//ELP_SyncHead(serX,serY,1);
			//jalan2(150,50000);
        		//ELP_SyncHead(serX,serY,1);
			//jalan2(150,50000);
			//usleep(70000);
			//siapJalan();
			//work 3/21/2015
			//set_siap_jalan(70,70,7,7,10,10,0,0,0,0,0,0);
			//set_parameterjalan(20,20,10,10,0,0,0,0,0,0,0,0);

//			set_parameterjalan(-15,-15,12,12,10,10,0,0,0,0,0,0);

//			set_siap_jalan(70,70,7,7,15,15,0,0,0,0,0,0);
                        //set_parameterjalan(10,10,12,12,10,10,0,0,0,0,0,0);
			//sleep(1);
			//jalan2(170,50000);
			//jalan2(170,50000);
			//stabilizer1(1,(serY-512),0.1,0.1,0,0,0,0);
	//		jalan2(170,50000);
			//resetStabilizer(1);
	//		sleep(5);
	//		jalan2(170,50000);
	//		jalan2(170,50000);
	//		jalan2(170,50000);
	//		jalan3(100,15500); //fase ending
			//siapJalan();
			set_parameterjalan(25,25,20,20,7,7,0,0,0,0,0,0);
			//fixedPID();
			usleep(500000);//1000000
                        //usleep(1000000);
//	usleep (1000);
  //      ELP_SyncHead(512,512,1);
//	serX=512; serY=512; usleep (1000);
    //resetStabilizer(2);
    			break;

		case 6 :
			Lshoot();
			usleep(1000000);
			siapJalan(1,0);
			break;

		case 7:
			setAllTorque(1000);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			siapJalan(1,0);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
			usleep(70000); 
			printf ("razi super shoot -_- \n");
            		TendangMaret2015();
			usleep(50000);
			siapJalan(1,0);
//                        set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
            		usleep(1000000);
           		break;		
		
		case 8:
			JatohKanan2016();
			//bangun_depan();
			break;

		case 9:
			JatohKiri2016();
			//bangun_depan();
			break;

	        case 28 :
                       //set_siap_jalan(0,0,0,0,28,3,0,0,0,0,0,0);
		       pid(277);
			siapJalan(1,pd);
		       pid(277);
                       //set_siap_jalan(0,0,0,0,-28,-3,0,0,0,0,0,0);
		       usleep(1000);
		       break;
			
		case 18:
			//ELP_SyncHead(512,512,1);
            		bangun_depan(); //setAllSpeed(20);sleep (1);
	    		reset_siap_jalan();
			//siapJalan2(1,0,10); 
			siapJalan(1,0);
			pid(283);
			sleep (1);
			break;

		case 19:
			//ELP_SyncHead(512,512,1);
		      	cout<<"belakang1"<<endl;
			bangun_belakang1();
			sleep(1);
			cout<<"belakang2"<<endl;
			bangun_belakang2();
			sleep(1);
			cout<<"belakang3"<<endl;
			bangun_belakang3(); 
			sleep (1);
			bangun_depan_laststep();
			pid(277);
			//reset_siap_jalan();
			//set_siap_jalan(0,0,0,0,-3,-3,0,0,0,0,0,0);
	    		//siapJalan(1,0);
                        //set_siap_jalan(0,0,0,0,3,3,0,0,0,0,0,0);
	    		//fixedPID();
	    		sleep (1);
			break;
        

		case 51:
			setAllTorque(1000);
			set_parameterjalan(-25,-25,-20,-20,-15,-15,0,0,0,0,-4,-4);
			set_siap_jalan(0,0,0,0,-20,-20,0,0,0,0,0,0);
			setAllSpeed(170);
			pid(285);
			siapJalan(1,pd);
			usleep(1000000);
			jalan2(190,42000);
			//usleep(80000);
			//reset_siap_jalan();
			//usleep(4000);
			pid(283);
			//reset_siap_jalan();
			siapJalan(1,pd);
			set_siap_jalan(0,0,0,0,20,20,0,0,0,0,0,0);
			set_parameterjalan(25,25,20,20,15,15,0,0,0,0,4,4);
			break;
		
		case 29: 
			//setAllSpeed(200);
			//setAllTorque(1023);
			sitGK();
			break;

	}
	usleep(200000);
}

#endif
