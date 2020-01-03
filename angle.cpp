#include <ros/ros.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "math.h"
using namespace cv;
using namespace std;

int main(int argc,char** argv)
{   
   Mat dst,frame,cdst,cdstP;
   VideoCapture cap("/home/amit/Videos/pq_gate.mp4");
  while(1)
  {
    cap>>frame;
    waitKey(50);
    inRange(frame,Scalar(0,106,240),Scalar(176,255,255),frame);
    Mat element = getStructuringElement(MORPH_CROSS,Size(5,5),Point(2,2) );
    erode(frame,frame,element);
    blur(frame,frame,Size(3,3),Point(-1,-1));
    dilate(frame,frame,element);
    Canny(frame,dst,50,200,3);  
    cvtColor(dst,cdst,COLOR_GRAY2BGR);
    cdstP=cdst.clone();
    vector<Vec4i> linesP;
        HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
    // Draw the lines
    int maxx=0,minn=180,a;
    float m1,m2,angle=0,a1,a2;    
    for( size_t i = 0; i < 4; i++ )
    {
        Vec4i l = linesP[i];
        if(l[2]!=l[0]) {
        float slope=(float)(l[3]-l[1])/(l[2]-l[0]);
        a=-1*atan(slope)*(180/CV_PI);
        }
	else 
        a=90;
        maxx=max(maxx,a);
        minn=min(minn,a);    
        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    }
       angle=maxx+minn;
       ROS_INFO("ANGLE : %f",angle);
        imshow("Source", frame);
    imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
  }
}
