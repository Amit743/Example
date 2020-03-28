#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <hammerhead/hammerhead.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <pid_controller/Setpoint.h>
#include <chrono>
#define think 1
using namespace cv;

image_transport::Publisher frontDebugPub;
sensor_msgs::ImagePtr frontDebugMsg;
typedef std::chrono::system_clock::time_point TimePoint;
typedef std::chrono::high_resolution_clock Clock;
TimePoint search_time,inc_depth,lastFrontCameraCallbackTime;
TimePoint time_duration = Clock::now();
int time_out = 60;
Mat orig, tt, houghL, houghR, lp, rp;
float sway=0,surge=0,heave=0;
 int iLowH = 20;
 int iHighH = 30;

 int iLowS = 100;
 int iHighS = 255;

 int iLowV = 100;
 int iHighV = 255;
 
 int x = 0;

double dst(Point p1, Point p2)
{
    double d = (p1.x - p2.x)*(p1.x - p2.x) +(p1.y - p2.y)*(p1.y - p2.y);
    double dist = pow(d, 0.5);
    return dist;
}

Point getCentre(std::vector<Vec4i> l,int rows)
{
    int x_avg=0,y_avg=0;
    static int retX,retY,flag=1;
    double lineLen = 0;
    for(int i=0;i<l.size();i++)
    {
        double len = dst(Point(l[i][0],l[i][1]),Point(l[i][2],l[i][3]));
        x_avg+=len*(l[i][0]+l[i][2])/2;
        y_avg+=len*(l[i][1]+l[i][3])/2;
        lineLen+=len;
    }
    if(l.size()!=0)
    {
        x_avg/=lineLen;
        y_avg/=lineLen;
        //if(abs(x_avg-retX)<20 || flag==1)
            retX = x_avg;
        //if(abs(y_avg-retY)<5 || flag==1)
            retY = y_avg;
        flag=0;
    }
    return Point(retX,retY);
}

void frontCameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    lastFrontCameraCallbackTime = Clock::now();
    try
    {
        if(!cv_bridge::toCvShare(msg, "bgr8")->image.empty())
        {
            tt=cv_bridge::toCvShare(msg, "bgr8")->image;
            orig = tt.clone();
            // imshow("Raw",orig);
        }
        else
        {
            AUV_ERROR("Image is empty");
            return;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        AUV_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        return;
    }
}

double findTimeDifference(TimePoint end, TimePoint start)
{
    return ((double)std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count())*0.001;
}

bool detectTask(Mat src)
{
    if(src.empty())
    {
        return false;
    }
    Mat temp;
    temp=src.clone(); 

        cvtColor(src, src, COLOR_BGR2HSV);
        
        inRange(src, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), src);
          // imshow("srcBefore", src);
          erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
          dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
          dilate(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
          erode(src, src, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
          imshow("srcAfter", src);

        lp = src(Rect(src.cols*(0), src.rows*(0), src.cols*(0.5), src.rows*(1)));
        rp = src(Rect(src.cols*(0.5), src.rows*(0), src.cols*(0.5), src.rows*(1)));

        
        std::vector<Vec4i> linesPXL,linesPXR,verticalL,verticalR;
        HoughLinesP(lp, linesPXL, 1, CV_PI/180, 50, 50, 10 );
        HoughLinesP(rp, linesPXR, 1, CV_PI/180, 50, 50, 10 );
        houghL = Mat::zeros(lp.size(),CV_8UC3);
        houghR = Mat::zeros(rp.size(),CV_8UC3);
        for( size_t i = 0; i < linesPXL.size(); i++ )
        {
           Vec4i l = linesPXL[i];
           double angle = atan2(abs(l[1]-l[3]),abs(l[0]-l[2])) * 180 / M_PI;
           if(abs(90.0-angle) < 5)
           {
               line(houghL, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), 3, LINE_AA);
               verticalL.push_back(linesPXL[i]);
           }
        }
        for( size_t i = 0; i < linesPXR.size(); i++ )
        {
           Vec4i l = linesPXR[i];
           double angle = atan2(abs(l[1]-l[3]),abs(l[0]-l[2])) * 180 / M_PI;
           if(abs(90.0-angle) < 5)
           {
               line(houghR, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), 3, LINE_AA);
               verticalR.push_back(linesPXR[i]);
           }
        }
        // imshow("houghL", houghL);
        // imshow("houghR", houghR);
        waitKey(1);
        Point lx=getCentre(verticalL,src.rows),rx=getCentre(verticalR,src.rows);
        if(lx.x>src.cols*0.1 && rx.x>src.cols*0.1 )
        {
            rx.x+=src.cols/2;
            if(dst(lx,rx)>src.cols*0.4)
            {
                circle( temp, lx, 2, Scalar( 0, 0, 255 ), 3, LINE_AA );
                circle( temp, rx, 2, Scalar( 0, 0, 255 ), 3, LINE_AA );
                Point detect = Point((lx.x+rx.x)/2,src.rows/2);
                circle( temp, detect, 2, Scalar( 255, 255, 0 ), 3, LINE_AA );
                if(detect.x<src.cols*0.4)
                    {
                        // std::cout<<"Here1\n";
                    sway=0;surge=15;
                }
                else if(detect.x>src.cols*0.6)
                    {
                        // std::cout<<"Here2\n";
                        sway=0;surge=15;
                    }
                else
                    {
                        // std::cout<<"Here3\n";
                        sway=0;surge=15;
                    }
                // imshow("gate",temp);
                return true;
            }
        }
        else if(lx.x>src.cols*0.1 && rx.x==0)
        {
            // std::cout<<"Here4\n";
            sway=0;surge=2;
            // imshow("gate",temp);
            return true;
        }
        else if(lx.x==0 && rx.x>src.cols*0.1)
        {
            // std::cout<<"Here5\n";
            sway=0;surge=2;
            // imshow("gate",temp);
            return true;
        }

        frontDebugMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp).toImageMsg();
        frontDebugPub.publish(frontDebugMsg);
    return false;
}

int main(int argc, char **argv)
{    
    int DEPTH;
    ros::init(argc, argv, "qualification_task");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    frontDebugPub = it.advertise("/front_camera/debug", 1);
    ros::Publisher setpointPub = nh.advertise<pid_controller::Setpoint>("/setpoints", 1);
    ros::Subscriber frontCameraSub = nh.subscribe("/front_camera/image_rect_color",1,&frontCameraCallback);
    
    // namedWindow("Trackbars");
    // createTrackbar( "Canny low H", "Trackbars", &iLowH, 179, NULL);
    // createTrackbar( "Canny high H", "Trackbars", &iHighH, 179, NULL);
    // createTrackbar( "Canny low S", "Trackbars", &iLowS, 255, NULL);
    // createTrackbar( "Canny high S", "Trackbars", &iHighS, 255, NULL);
    // createTrackbar( "Canny low V", "Trackbars", &iLowV, 255, NULL);
    // createTrackbar("Canny high V", "Trackbars", &iHighV, 255, NULL);
    
    int run_duration = atoi(argv[1]);
    DEPTH = atoi(argv[2]);
    pid_controller::Setpoint sp;
    for(int i = 0;i < 6; i++)
    {
        sp.setpoints.push_back(0);
    }
    
    // enum movement_mode{FORWARD,LEFT_SWAYA,LEFT_SWAYB,RIGHT_SWAYA,RIGHT_SWAYB};
    // movement_mode move_state;
    
    ros::Rate loop_rate(15);
    bool initial_hardforward=true;
    search_time = Clock::now();
    inc_depth = Clock::now();
    sway = 0; surge = 15; heave = DEPTH;
    while (ros::ok())
    {
        if(findTimeDifference(Clock::now(),inc_depth) > 1)
        {
            heave+=1;
            if(heave>1008)
                heave=1008;
            inc_depth = Clock::now();
        }
        if(initial_hardforward)
        {
            if(findTimeDifference(Clock::now(),search_time) > run_duration)
            {
                initial_hardforward = false;
            }
        }
        else
        {
            sway = 0; surge = 0; heave = 1000; 
 	       exit(0);
        }
//        waitKey(1);
        
//        sp.setpoints[5] = 90;
        sp.setpoints[0] = surge;
        sp.setpoints[1] = sway;
        sp.setpoints[2] = heave;
        loop_rate.sleep();
        setpointPub.publish(sp);
        ros::spinOnce();
    }
    return 0;
}
