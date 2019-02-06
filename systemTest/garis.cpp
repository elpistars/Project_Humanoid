#ifndef GARIS_CPP
#define GARIS_CPP

#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/bola1.cpp"

int b_garis,c_garis,all_garis;

int korx_garis, kory_garis, area_garis;
int p1_garis=0;
int p2_garis=0;

void contour(Mat image);

void gerak(){
	p1_garis=0;
	
	if(korx_garis<125){
		b_garis=a_Image+10;
		a_Image=b_garis;
			if ( a_Image <= 0 ){
			a_Image = 0;
			}
			else if ( a_Image >= 1023 ){
			a_Image = 1023;
			}
		int id[] = {20};
					int data1[][18] = {a_Image};
					int delay1[] = {1000};
					int speed1[] = {100};
					kirimPacketGerak(1, 1, 1, id, data1, 1, delay1, speed1);
					if(modeputar==0){
					comimage=3;}
	}
	else if(korx_garis>320){
		c_garis=a_Image-10;
		a_Image=c_garis;
			if ( a_Image <= 0 ){
			a_Image = 0;
			}
			else if ( a_Image >= 1023 ){
			a_Image = 1023;
			}
		int id[] = {20};
					int data1[][18] = {a_Image};
					int delay1[] = {1000};
					int speed1[] = {100};
					kirimPacketGerak(1, 1, 1, id, data1, 1, delay1, speed1);
					if(modeputar==0){
					comimage=2;}
	}
	else{
			if ( a_Image <= 0 ){
			a_Image = 0;
			}
			else if ( a_Image >= 1023 ){
			a_Image = 1023;
			}
		int id[] = {20};
					int data1[][18] = {a_Image};
					int delay1[] = {1000};
					int speed1[] = {100};
					kirimPacketGerak(1, 1, 1, id, data1, 1, delay1, speed1);
	if(modeputar==0){		
	comimage=1;}
	}
	cout<<korx_garis<<endl;
	
}

 void penggaris()
 {
	modegaris=0;
	
   
//cap.read(imgOriginal);
  Mat imgHSV;

  cvtColor(imgOriginal, imgHSV, COLOR_RGB2GRAY); //Convert the captured frame from BGR to HSV
  
  Mat imgThresholded;
  threshold(imgHSV,imgThresholded,133,255,0);
  //inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

  //morphological opening (remove small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

   //morphological closing (fill small holes in the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  Moments moment = moments (imgThresholded);
 

  contour(imgThresholded);
  
		if(p1_garis==1){
		gerak();
	
		}
		
		else if(p2_garis==1){
		p2_garis=0;
		if(modeputar==0){
		comimage=0;}
		}
	}




 void contour(Mat gray)
 {
     
	 Canny(gray, gray, 100, 200, 3);

     /// Find contours
     vector<vector<Point> > contours;
	 
	 vector<Vec4i> hierarchy;
     RNG rng(12345);
     findContours( gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

     /// Draw contours
	 Mat drawing = Mat::zeros( gray.size(), CV_8UC3 );

	 for(unsigned int i=0; i<contours.size(); i++)
	{

		for(unsigned int j=0; j<contours[i].size(); j++)
		{
			if (i==0 && j>50 && j<53){
			p1_garis=1;
			korx_garis = contours[i][j].x;
			kory_garis = contours[i][j].y;
			circle(drawing,cvPoint(korx_garis,kory_garis),20,100,3,2,0);
			line(drawing,cvPoint(273,0),cvPoint(273,480),255,2,30,0); //garis pada layar
			line(drawing,cvPoint(366,0),cvPoint(366,480),255,2,30,0); // garis pada layar
			line(drawing,cvPoint(0,180),cvPoint(640,180),255,2,30,0);
			line(drawing,cvPoint(0,300),cvPoint(640,300),255,2,30,0); // garis pada layar
			}
			
			else{
			p2_garis=1;
			}

		}

		for( int i = 0; i< contours.size(); i++ )
		{

         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );

         ////////////////////
           vector<Moments> mu(contours.size() );
           for( int i = 0; i < contours.size(); i++ )
               { mu[i] = moments( contours[i], false ); }

            ///  Get the mass centers:
            vector<Point2f> mc( contours.size() );
            for( int i = 0; i < contours.size(); i++ )
               { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

		}


	}
 }
#endif
