#ifndef BOLA1_CPP
#define BOLA1_CPP

#include "/home/pi/systemTest/library/allLib.h"
VideoCapture cap(0); //capture the video from web cam
int FRAME_WIDTH=640;
int FRAME_HEIGHT=480;
Mat matProcessed; // Variable matriks untuk kamera
Mat matWebcam2; // Variable matriks untuk kamera
Mat imgOriginal;
Mat matWebcam;
double a_Image;
double b_Image;
double error_sblmIx_Image ;
double error_sblmDx_Image;
const double Tc_Image = 0.01 ;
const double Kpx_Image = 0.002;
const double Kdx_Image = 0.0006;
const double Kix_Image = 0 ;
double error_sblmIy_Image ;
double error_sblmDy_Image;
const double Kpy_Image = 0.002;
const double Kdy_Image = 0.0006;
const double Kiy_Image = 0;
double setpointx_Image;
double setpointy_Image;
double nilai_sensorx_Image;
double nilai_sensory_Image;
double errorx_Image;
double errory_Image;
double outPx_Image;
double outPy_Image;
double outIx_Image;
double outIy_Image;
double outDx_Image;
double outDy_Image;
double PIDx_Image;
double PIDy_Image;
double errorIx_Image;
double errorIy_Image;
double errorDx_Image;
double errorDy_Image;
double sudut_Image;
int comimage,modegaris;
double area_Image;
int kory_Image=640;
int korx_Image=480;
int modeputar=0;
void balls(){
	cap >> imgOriginal;
	cap >> matWebcam2; // variable cap yang di ubah menjadi matwebcam2
	cvtColor(imgOriginal,imgOriginal,CV_BGR2Lab); //pengubahan dari kamera normal (BRG) menjadi HSV inputnya matwebcam dan outpunya juga matwebcam
	inRange(imgOriginal,Scalar(37,133,0),Scalar(158,256,256),matProcessed);
	GaussianBlur(matProcessed,matProcessed,Size(9,9),1.5);	//ngeblur warna pada kamera yang sudah terproses agar mudah di deteksi
	Moments moment = moments (matProcessed); // fungsi moment untuk mengetahui luas objek yang terdeteksi

	area_Image = moment.m00;

	if(area_Image >= 80000){
		korx_Image = moment.m10/area_Image;
		kory_Image = moment.m01/area_Image;
		if(modeputar==0){
		comimage=1;}
		modegaris=0;
		}
	else{
		a_Image=512;
		b_Image=215;
		if(modeputar==0){
		comimage=0;}
		PIDx_Image=0;
		PIDy_Image=0;
		    error_sblmDx_Image=0;
        	error_sblmDy_Image=0;
	        korx_Image=640;
	        kory_Image=480;
			modegaris=1;
	}

	 circle(matProcessed,cvPoint(korx_Image,kory_Image),5,100,3,2,0); // titik tengah luas yang terdeteksi
	 circle(matProcessed,cvPoint(320,240),5,100,3,2,0);
	 
	cout<<" X = "<<korx_Image<<" Y = "<<kory_Image<<"  area_Image : "<<area_Image<<endl;
	if(kory_Image>=390){
		modeputar=1;
	comimage=4;}
	/*
//PIDx_Image
	setpointx_Image= FRAME_WIDTH/2;
	nilai_sensorx_Image = korx_Image;
	    
//P
	errorx_Image=setpointx_Image-nilai_sensorx_Image;
	outPx_Image=Kpx_Image*errorx_Image;

//D
	
	errorDx_Image=errorx_Image-error_sblmDx_Image;
	outDx_Image=Kdx_Image*errorDx_Image;
	error_sblmDx_Image = errorx_Image;

	PIDx_Image=outPx_Image+outDx_Image;
	
//PIDy_Image
	setpointy_Image=FRAME_HEIGHT/2; 
	nilai_sensory_Image=kory_Image;

//P
	errory_Image=setpointy_Image-nilai_sensory_Image;
	outPy_Image=Kpy_Image*errory_Image;


//D
	
	errorDy_Image=errory_Image-error_sblmDy_Image;
	outDy_Image=Kdy_Image*errorDy_Image;
	error_sblmDy_Image=errory_Image;

	PIDy_Image=outPy_Image+outDy_Image;

	
	a_Image=a_Image+PIDx_Image;
	b_Image=b_Image+PIDy_Image;
	/*
	int id[] = {19, 20};
    int data1[][18] = {215, 512};
    int delay1[] = {1000, 1000};
    int speed1[] = {100, 100};
    kirimPacketGerak(2, 1, 1, id, data1, 1, delay1, speed1);*/

}
#endif
