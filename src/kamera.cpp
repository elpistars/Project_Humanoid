
// Threshold CIE.cpp : Defines the entry point for the console application.
//

#include <algorithm>
#include <thread>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
using namespace std;
using namespace cv;
struct imgGaris{
  int kanan,kiri,tengah;
}garis;
//std::mutex mutex_garis;
VideoCapture cap(0); //capture the video from web cam

void hough_draw(Mat src, Mat &dst);
void init_cam(){

    cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
    }
}
void read_cam(){
//	std::lock_guard<std::mutex> lockGuard(mutex_garis);
        Mat imgOriginal;
    	int iLowL = 98;
	int iHighL = 182;
    	int iLowa = 103; 
    	int iHigha = 194;
    	int iLowb = 102;
    	int iHighb = 183;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
        }

	Mat imgHSV, imgLAB;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2Lab);
        imgLAB=imgHSV.clone();
        Rect Rec((FRAME_WIDTH/2)-100,380,200,100);
        rectangle(imgHSV,Rec,Scalar(100,100,100),1,8,0);
        Mat imgThresholded;
        Mat imgOutput=imgOriginal.clone();
        imgHSV=imgHSV(Rec);
        inRange(imgHSV, Scalar(iLowL, iLowa, iLowb), Scalar(iHighL, iHigha, iHighb), imgThresholded); //Threshold the image

  //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

   //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        rectangle(imgOutput,Rec,Scalar(255,0,0),1,8,0);
        imgOutput=imgOutput(Rec);
        hough_draw(imgThresholded,imgOutput);
        //thresh_callback(0,0,imgThresholded);
        //Mat imcontrol;
        //Mat imcontrol_LAB_L(100,300,CV_8UC3,Scalar(iLowL,iLowa,iLowb));
        //cvtColor(imcontrol_LAB,imcontrol,COLOR_Lab2BGR);
        //imshow("Control", imcontrol_LAB_L);
}

void hough_draw(Mat src,Mat &dst){
  Mat canny_output;
  Point line_pos[2];
  int tengah;
  int line_count=0;
  Canny(src,canny_output,50,200,3);
  vector<Vec2f> lines;
  HoughLines(canny_output,lines, 1,CV_PI/180,50,0,0);
  for (size_t i=0; i<lines.size();i++){
      float rho = lines[i][0], theta=lines[i][1];
      Point pt1, pt2;
      double a=cos(theta), b=sin(theta);
      double x0=a*rho, y0 =b*rho;
      pt1.x=cvRound(x0+120*(-b));
      pt1.y=cvRound(y0+120*(a));
      pt2.x=cvRound(x0-120*(-b));
      pt2.y=cvRound(y0-120*(a));
   //   cout<<"i="<<i<<" "<<"pt1 : "<<pt1.x<<endl<<"pt2 : "<<pt2.x<<endl<<endl;
      if(max(pt1.x,pt2.x)<(src.size().width/2)){
        line_pos[0].x=max(max(pt1.x,pt2.x),line_pos[0].x);
      }
      else if(min(pt1.x,pt2.x)>(src.size().width/2)){
        line_pos[1].x=max(min(pt1.x,pt2.x),line_pos[1].x);
      }
  }
  //cout<<"kiri :"<<line_pos[0]<<endl<<"kanan :"<<line_pos[1]<<endl;
  if((line_pos[0].x)==0&&(line_pos[1].x!=0)) tengah=1;  
  else if((line_pos[1].x==0)&&(line_pos[0].x!=0)) tengah=2;
  else tengah=(line_pos[0].x+line_pos[1].x)/2;
  //cout<<tengah<<endl;
  garis.kiri=line_pos[0].x;
  garis.kanan=line_pos[1].x;
  garis.tengah=tengah;
//  cout<<garis.kiri<<" "<<garis.tengah<<" "<<garis.kanan<<endl;
  return;
}
