#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/gerakan.cpp"
#include <math.h>
#include <time.h>
#include <sys/time.h>

using namespace cv;
using namespace std;

#define P_GOAL_POSITION 30
#define _USE_MATH_DEFINES
int tempX,tempY;
float kec_kepala;

int counter;
float pidX,pidY,minSpeed;
const string windowName = "Kamera"; 
const int FRAME_WIDTH = 384; 
const int FRAME_HEIGHT = 288; 
int Matriks;
int temp;
float distRL;
int serX = 512; 
int serY = 512; 
clock_t waktu = clock();
float last_value;
int command=0;
int commandMotor=0;
//int id[2]={19,20}; 
int pos[2]={serX,serY}; 
int pos2[2]={600,600}; 
double area; 
int x=0; 
int y=0; 
int statX; 
int statY; 
int lastX;
int lastY;
int commandFind=0;
int loseBall;
int commandSearch=0;
int commandCam=0;
int comms=0;
int commandMove=0;
int moving=0;
int pengecekan=0;
VideoCapture cap(0);
pthread_mutex_t locking,locking2,locking3,locking4;

Mat webcam; 
Mat hsv;
Mat thresh;

int Hmin=0;
int Smin=139;
int Vmin=161;
int Hmax=21;
int Smax=255;
int Vmax=255;

void kirimPacket(int totalServo, int id[],int data[],int datapers); 
void EPMoments(Mat thresh1); 
int pixelmap(int x, int y);
void morph(Mat thresh2);


void kirimPacket(int totalServo, int id[],int data[],int datapers){
    int l=datapers;
    int n=totalServo;
    int i=0;
    dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
    //dxl_set_txpacket_length(); //jumlah paket yang dikirim 0x18
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
	dxl_set_txpacket_parameter(0,P_GOAL_POSITION);//yang mau ditulis 0x1E
    dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
    for( i; i<totalServo;i++){
        dxl_set_txpacket_parameter(2+3*i+0,id[i]);
        dxl_set_txpacket_parameter(2+3*i+1,dxl_get_lowbyte(data[i]));
        dxl_set_txpacket_parameter(2+3*i+2,dxl_get_highbyte(data[i]));
    }
    dxl_set_txpacket_length((l+1)*n+4);
    dxl_txrx_packet();
}




 void morph(Mat thresh2){
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
    erode(thresh2,thresh2,erodeElement);
    dilate(thresh2,thresh2,dilateElement);
}

 void EPMoments(Mat thresh1){
    float pX,pY,dX,dY,iX,iX1,iY,iY1,rate_X,rate_Y,error_X,error_Y,degree,tanteta,dist;
    //int temp;
    float intercept = -56.994;
    float slope = 0.254221;
    float intercept1 = 0.084776;
    float slope1 = 0.094198;

    int sp_X=FRAME_WIDTH/2;
    int sp_Y=FRAME_HEIGHT/2;
    double refArea = 10000;
    float KpX=0.0585;
    float KpY=0.04;
    //float KpY=0.0325;15
    float KdX=0.02;
    float KdY=0.03;
    //float KdY=0.045;15
    //float KiX=0.01;
    //float KiY=0.0125;

    int tinggi = 30;

    Moments moment = moments (thresh1);
    area = moment.m00;
    if(area<refArea){
	//pid(273);
	moving=0;
	area=moment.m00;
	//imshow(windowName,thresh);
	 //if(waitKey(30)>=0){}
	if(loseBall==15){commandSearch=0; moving=0;sleep(2);loseBall=0;}
	else if(area<refArea){loseBall++;usleep(5000);
		pos[1]=pos[1]+pidX+statX;
	        pos[0]=pos[0]+pidY+statY;
	        kirimPacket(2,idh,pos,2);
		if(pos[0] <= 0){pos[0]=0;}
	        if(pos[0] >= 512){pos[0]=512;}
        	if(pos[1] >= 650){pos[1]=650;}
        	if(pos[1] <= 262){pos[1]=262;}
                area=moment.m00;

	}
    }
    if(area >= refArea){
		//pid(175);
		loseBall=0;
		x = moment.m10/area;
	    	y = moment.m01/area;

		if (pixelmap(x,y) != 0)
	        temp = pixelmap(x,y);

	switch (temp){

		case 11 : statX= 16; statY= 10; break;
		case 12 : statX= 0; statY= 10; break;
		case 13 : statX=-16; statY= 10; break;
		case 21 : statX= 16; statY= 0; break;
		case 22 : statX= 0; statY= 0; break;
		case 23 : statX=-16; statY= 0; break;
		case 31 : statX= 16; statY=-10; break;
		case 32 : statX= 0; statY=-10; break;
		case 33 : statX=-16; statY=-10; break;
        }


        //PID
        error_X=sp_X-x; //Error Value
        error_Y=sp_Y-y; //Error Value

	serX=serX+statX; //Horizontal Servo Value for Future
        serY=serY+statY; //Vertical Servo Value for Future

        pX=error_X*KpX;
        pY=error_Y*KpY;


        if (statX==0) pX=0;
      	if (statY==0) pY=0;

        rate_X=error_X-lastX;
       	rate_Y=error_Y-lastY;

       	dX=rate_X*KdX;
       	dY=rate_Y*KdY;

	/*iX1=error_X+lastX;
	iY1=error_Y+lastY;

	iX=iX1*KiX;
	iY=iY1*KiY;*/

       	pidX=pX+dX;
       	pidY=pY+dY;

        pos[1]=serX+pidX;
       	pos[0]=serY+pidY;

	lastX=error_X;
        lastY=error_Y;

	if(pos[0] <= 0){pos[0]=0;}
	if(pos[0] >= 512){pos[0]=512;}
        if(pos[1] >= 650){pos[1]=650;}
	if(pos[1] <= 262){pos[1]=262;}

	kirimPacket(2,idh,pos,2);

        cout<<"Area: "<<area<<endl;
	kirimPacket(2,idh,pos,2);

	degree = 180*(pos[0]-208)/512;
        tanteta = tan(degree*M_PI/180);
        dist = tanteta*tinggi;
	distRL = intercept + slope*pos[0];
	waktu = (clock()-waktu)/1000;
	kec_kepala=((last_value-distRL)*10000)/waktu;
	usleep(1000);
	//printf ("bola : (%d,%d) servo : (%d,%d) \n Jarak : (%0.1f)",x,y,pos[0],pos[1],distRL);
	//cout << "Bola : (" <<x<<","<<y<<") Servo : ("<<pos[0]<<","<<pos[1]<<")"<<endl;
        minSpeed = intercept1 + slope1*distRL;
	cout << "Jarak : (" << distRL << " cm) Kecepatan : (" << kec_kepala << "cm/ms) Kecepatan Minimum : (" << minSpeed << ")" << endl;
	last_value = distRL;
	//usleep(10000);
	}
}

int pixelmap (int x, int y) {

	//int Matriks=0;

	if (( x<=FRAME_WIDTH/3 ) && (y<=FRAME_HEIGHT/3)) 				Matriks = 11; else
	if (((x<=FRAME_WIDTH/3*2) && (x>FRAME_WIDTH/3)) && (y<=FRAME_HEIGHT/3)) 	Matriks = 12; else
	if (((x<=FRAME_WIDTH ) && (x>FRAME_WIDTH/3*2)) && (y<=FRAME_HEIGHT/3)) 		Matriks = 13; else
	if (( x<=FRAME_WIDTH/3 ) && (y<=FRAME_HEIGHT/3*2)) 				Matriks = 21; else
	if (((x<=FRAME_WIDTH/3*2) && (x>FRAME_WIDTH/3)) && (y<=FRAME_HEIGHT/3*2)) 	Matriks = 22; else
	if (((x<=FRAME_WIDTH ) && (x>FRAME_WIDTH/3*2)) && (y<=FRAME_HEIGHT/3*2)) 	Matriks = 23; else
	if (( x<=FRAME_WIDTH/3 ) && (y<=FRAME_HEIGHT))				 	Matriks = 31; else
	if (((x<=FRAME_WIDTH/3*2) && (x>FRAME_WIDTH/3)) && (y<=FRAME_HEIGHT)) 		Matriks = 32; else
	if (((x<=FRAME_WIDTH ) && (x>FRAME_WIDTH/3*2)) && (y<=FRAME_HEIGHT)) 		Matriks = 33;

	return (Matriks);

}

void searching2(){

}

void* searching(void *arg){
	if(command==0){
		readWifi();
		//kasus(28);
		pthread_mutex_lock(&locking2);	
		//moving=1;
		//kasus(28);
		commandSearch=0;
		int k=0;
		double refArea=15000;
		bool search=1;
		
		if(commandSearch==0){
			dxl_write_word(idh[0],P_GOAL_POSITION,searchingY[0]);
		}
		moving=0;
        	sleep(1);
		
		if(commandSearch==0){
			for(int i=0;i<9;i++){
				readWifi();
				cout<< serY<<endl;
				if(commandSearch==1){serX=searchingX[i]; serY=searchingY[0];}
				if(command==1){i=100;}
				dxl_write_word(idh[1],P_GOAL_POSITION,searchingX[i]); 
				if(commandSearch==1){serX=searchingX[i];serY=searchingY[0];}
				if(command==1){i=100;}

				for(int j=0;j<5;j++){
					readWifi();
					usleep(10000);
					cap.read(webcam);
					cvtColor(webcam,hsv,COLOR_BGR2HSV);
					inRange(hsv,Scalar(Hmin,Smin,Vmin),Scalar(Hmax,Smax,Vmax),thresh);
					morph(thresh);
					Moments moment = moments (thresh);
					area=moment.m00;
					cout<<"Area: "<<area<<endl;
					//imshow(windowName,thresh);
					//if(waitKey(30)>=0){}
				}
            
            if(area>=refArea){commandSearch=1;}
			k=i;
			if(commandSearch==1){serX=searchingX[i];serY=searchingY[0];cout<<"array ke: "<<i<<endl;i=100;}
		
	if(command==1||WIFI==0){i=100;}
			
			/*cap1.read(webcam);
			cvtColor(webcam,hsv,COLOR_BGR2HSV);
			inRange(hsv,Scalar(4,123,91),Scalar(11,255,255),thresh);
			morph(thresh);
			Moments moment = moments (thresh);
			area=moment.m00;
                        imshow(windowName,webcam);
			if(waitKey(30)>=0){break;}
			if(area>=refArea){break;}
			usleep(100);*/
			}
		}
		//if(area>=refArea){break;}
        if(commandSearch==0){cout<<"masuk"<<endl;
			dxl_write_word(idh[0],P_GOAL_POSITION,searchingY[1]);
		}
        
		if(commandSearch==0){
			cout<<"masuk1"<<endl;
			for(int i=8;i>=0;i--){
				readWifi();
				if(commandSearch==1){serX=searchingX[i];serY=searchingY[1];}
			
				if(command==1){i=-1;}
                		dxl_write_word(idh[1],P_GOAL_POSITION,searchingX[i]);
				
				if(commandSearch==1){serX=searchingX[i];serY=searchingY[1];}
				if(command==1){i=-1;}
				//usleep(1000000);
                
				for(int j=0;j<5;j++){
					readWifi();
                    			usleep(10000);
                    			cap.read(webcam);
                    			cvtColor(webcam,hsv,COLOR_BGR2HSV);
					inRange(hsv,Scalar(Hmin,Smin,Vmin),Scalar(Hmax,Smax,Vmax),thresh);
                    			morph(thresh);
                    			Moments moment = moments (thresh);
                    			area=moment.m00;
                    			//imshow(windowName,thresh);
		    			cout<<"Area: "<<area<<endl;
                    			//if(waitKey(30)>=0){}
				}
                
				if(area>=refArea){commandSearch=1;}
				k=i;
                
				if(commandSearch==1){serX=searchingX[i];serY=searchingY[1];cout<<"array ke: "<<endl;i=-1;}
				if(command==1||WIFI==0){i=-1;}
			}
                
				/*cap1.read(webcam);
                cvtColor(webcam,hsv,COLOR_BGR2HSV);
                inRange(hsv,Scalar(4,123,91),Scalar(11,255,255),thresh);
                morph(thresh);
                Moments moment = moments (thresh);
                area=moment.m00;
				imshow(windowName,webcam);
                if(waitKey(30)>=0){break;}
				if(area>=refArea){break;}
				usleep(30);*/
		}
	}
	if(commandSearch==1){commandMove=0;}
	//commandSearch=0;	
	//kasus(2);
	//moving=2;
	//pos[0]=serX;
	//pos[1]=serY;
	//dxl_write_word(19,P_GOAL_POSITION,serX);
	//dxl_write_word(20,P_GOAL_POSITION,serY);
	pthread_mutex_unlock(&locking2);
}

void* readCam(void *arg){	
	//cout<<"Masuk Function"<<endl;
	int refArea=300000;
	//pthread_mutex_lock(&locking);
	//commandCam=1;
	readWifi();
	//VideoCapture cap1=((VideoCapture) arg);
	//cout<<"function"<<endl;
	cap.read(webcam);
	//cout<<"test"<<endl;
    cvtColor(webcam,hsv,COLOR_BGR2HSV);
	//cout<<"function3"<<endl;
    inRange(hsv,Scalar(Hmin,Smin,Vmin),Scalar(Hmax,Smax,Vmax),thresh);
	//cout<<"function4"<<endl;
	morph(thresh);
	Moments moment =  moments(thresh);
	//cout<<"function2"<<endl;
    EPMoments(thresh);
    kirimPacket(2,idh,pos,2);
    area=moment.m00;
	//imshow(windowName,thresh);
	//if(waitKey(30)>=0){}
	//if(area>=refArea){comms=1;}
	//if(waitKey(30)>=0){}
	//commandCam=0;
	//pthread_mutex_unlock(&locking);
}

void* find(void *arg){
	while(commandSearch==1&&WIFI==1){
	//pthread_mutex_lock(&locking4);
	commandFind=1;
	cout<<"masuk function"<<endl;
        cap.read(webcam);
	cout<<serX<<endl;
	//pos[0]=serX;
	//pos[1]=serY;
    cvtColor(webcam,hsv,COLOR_BGR2HSV);
    inRange(hsv,Scalar(Hmin,Smin,Vmin),Scalar(Hmax,Smax,Vmax),thresh);
    morph(thresh);
    EPMoments(thresh);
    //if(waitKey(30)>=0){}
    //kirimPacket(2,idh,pos,2);
    //imshow(windowName,thresh);
     //if(waitKey(30)>=0){}
	//kasus(5);
	//if(commandMove==0){
		//if(serX<419 && serY>290){moving=2;}
		//else if(serX>622){moving=2;}
		//else if(serY<=290){if(serX>622){moving=4;}else{moving=5;}}
		//else{moving=3;}
		//if(serY<=290){if(serX>419){moving=4;}else{moving=5;}}
	
	//if(commandMove==0&&commandCam==0){
		if(distRL>45){
			moving=0;
		}
		else if(distRL<=45){
			if(kec_kepala >= minSpeed ){
				if(serX<465){moving=9;}
				else if(serX>540){moving=10;}
				else{moving=0;}
				
			}
			//else{
			//	int arah=bacaMAG(Pmag_raw);
                        //      if(arah<234){moving=8;}
                        //        else if(arah>245){moving=7;}
			//	else{moving=5;}
			//}
		}
			/*if(serX>=560 && serX<=580 && serY<314 && serY>250){	moving=5;	}
			else if(serX<550 && serY>314){moving=8;}
			else if(serX<550 && serY<=314){moving=4;}
			else if(serX>622 && serY<=314){moving=2;}
			else if(serX>622 && serY>314){moving=7;}
			else{	moving=3;	}
		}*/
	//}
	if(command==1){moving=0;}
	//}
    
	//if(waitKey(30)>=0){}
	commandFind=0;
	//moving=0;
	//cout<<"Ngescan"<<endl;
	//pthread_mutex_unlock(&locking4);
	}
	commandFind=0;
}

void* moves(void *args){
	//pthread_mutex_lock(&locking3);
	if(commandSearch==1&&WIFI==1&&commandMove==0){
		readWifi();
		//commandCam=1;
		commandMove=1;
		cout<<"Moving"<<endl;
		//pthread_mutex_lock(&locking3);	
	//if(commandSearch==1&&WIFI==1){	
	        if(commandSearch==1&&WIFI==1&&command==0){	
		switch(moving){
			case 1: /*cout<<"Gerakan Standup"<<endl;break;*/kasus(29); break; 
			case 2: /*cout<<"Gerakan Geser Kanan"<<endl;break;*/kasus(3); break;
			case 4: /*cout<<"Gerakan Geser Kiri"<<endl;break;*/kasus(4); break;
			case 3: /*cout<<"Gerakan Jalan"<<endl;break;*/kasus(5); break;
			case 31:/*cout<<"Gerakan Jalan"<<endl;break;*/kasus(51);break;
			case 5: /*cout<<"Gerakan Tendang"<<endl;break;*/kasus(7); break;
			case 6: /*cout<<"Gerakan Bangun Belakang"<<endl;break;*/kasus(19); break;
			case 7: /*cout<<"Gerakan Putar Kanan"<<endl;break;*/kasus(1); break;
			case 8: /*cout<<"Gerakan Putar Kiri"<<endl;break;*/kasus(2); break;
			case 9: kasus(8); break;
			case 10: kasus(9);break;
		}
	//}
		cout<<serX<<", "<<serY<<endl;
		usleep(50000);
		//moving=1;
		//kasus(28);
		//commandCam=0;
		}
	}
	//commandMove=0;
	//pthread_mutex_unlock(&locking3);
	commandMove=0;
} 



int search(){
	//VideoCapture cap(0);
	cap.set(CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    	cap.set(CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	
	if(!cap.isOpened())
		return -1;
	
	if(dxl_initialize(0,1)==0){
        cout<<"Error connecting"<<endl;
        return 0;
    }
    
    else{
		cout<<"connect success!"<<endl;
	}
	kasus(28);
	usleep(100000);
	//kasus(29);
	while(1){
		pthread_t thread3, thread4, thread5,thread6;
		if(WIFI!=1){
			readWifi();
			commandSearch=0;
		}
		
		readWifi();
		if(WIFI==1&&commandSearch==0&&command==0){
			cout<<"Masuk searching"<<endl;
	//		while(!bacaMAG(Pmag_raw,117)&&command==0&&pengecekan==0){usleep(200000);kasus(28); if(putar==1){cout<<"putar kiri"<<endl;kasus(1);} else if(putar==2){cout<<"Putar Kanan"<<endl;kasus(2);}}
			if(pengecekan==0){kasus(29);}
			pengecekan=1;
			//siapJalan(1,-90);
			//kasus(2);
			//kasus(28);
			searching(NULL);
			if(commandSearch==0&&WIFI==1){/*kasus(2);*/}
		}

		if(counter>=60){
			commandMove=0;
			counter=0;
		}

		if(commandMove==1){
			counter++;
		}
		//kirimPacket(2,idh,pos,2);
		readWifi();
		if(WIFI==1&&commandSearch==1&&command==0&&commandMove==0){
			if(command==1){moving=0; commandSearch=0;}
			pthread_create(&thread3,NULL,moves,NULL);
			if(command==1){moving=0; commandSearch=0;}
		}
		readWifi();
		if(WIFI==1&&commandSearch==1&&command==0&&commandFind==0){
			cout<<"masuk ngikutin"<<endl;
			readWifi();
			//sleep(5);
			//cout<<serX<<", "<<serY<<endl;
			if(command==1){commandSearch=0;break;}
			if(WIFI==1&&commandSearch==1&&command==0&&commandFind==0){
			pthread_create(&thread4,NULL,find,NULL);
			}
			//find(NULL);
			if(command==1){commandSearch=0;break;}
			//if(area<50000){
			//commandCam=0;
			//while(commandCam=0){
			//kasus(28);
			//kasus(2);
			//searching(NULL);
			//}
			//}
		}
	}
}

