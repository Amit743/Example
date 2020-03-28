#include "hammerhead_control/MoveCmd.h"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <openvino_object_detection/Object.h>
#include <openvino_object_detection/Objects.h>
#include <ros/package.h>
using namespace cv;
int count = 0;
ros::NodeHandle *nh1;
int tstate = 0;
int countd = 0;
float dist_W=0,dist_H=0, avg_dist=0, sway=0, heave=0, avg_sway=0, avg_heave=0;
float avg_dist_t=0, avg_sway_t=0, avg_heave_t=0;
void process_next_image_fc(const sensor_msgs::ImageConstPtr &image_frame) {

  try {

      cv::Mat frame_front;

	      ros::ServiceClient ov_client =
	      nh1->serviceClient<openvino_object_detection::Objects>("/tiny_yolov3");
      openvino_object_detection::Objects t; 

      sensor_msgs::Image output_image_msg;
  
      hammerhead_control::MoveCmd a;

      cv_bridge::toCvShare(image_frame, "bgr8")->image.copyTo(frame_front);

    t.request.t = 0.5;
    t.request.iou_t = 0.8;
    t.request.img =
      *cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_front).toImageMsg();

  if (ov_client.call(t)) {

    for (openvino_object_detection::Object obj : t.response.objects) {
      std::ostringstream conf;
      // conf << ":" << std::fixed << std::setprecision(3) << obj.confidence;
  
        switch (obj.id){
        case 0:
          // std::cout<<"0\n";
          if(count<10){
            dist_H = (93*0.38)/(obj.h*0.00074);
            avg_dist = avg_dist + dist_H;
            avg_heave = avg_heave + ((abs(obj.y -244))*0.00074*dist_H)/0.38;
            avg_sway = avg_sway + ((abs(obj.x -324))*0.00074*dist_H)/0.38;
            count++;
          }

          break;
          return;
        case 1:
          // std::cout<<"1\n";
           dist_W = (61*0.38)/(obj.w*0.00074);
          dist_H = (93*0.38)/(obj.h*0.00074);
          avg_dist_t = avg_dist_t + (dist_H + dist_W)/2.0;
          countd++;
          break;
        case 2:
          // std::cout<<"2\n";
          dist_W = (61*0.38)/(obj.w*0.00074);
          dist_H = (98*0.38)/(obj.h*0.00074);
          avg_dist_t += (dist_H + dist_W)/2.0;
          // a.depth_time = 5;
          countd++;

          break;
        case 3:
          // std::cout<<"3\n";
          dist_W = (61*0.38)/(obj.w*0.00074);
          dist_H = (93*0.38)/(obj.h*0.00074);
          avg_dist_t += (dist_H + dist_W)/2.0;
          // a.depth_time = 5;
          countd++;

          break;
        case 6:
          // std::cout<<"6\n";
           dist_W = (58*0.38)/(obj.w*0.00074);
          dist_H = (93*0.38)/(obj.h*0.00074);
          avg_dist = (dist_H + dist_W)/2.0;
          // a.depth_time = 5;

          break;
        case 7:
          // std::cout<<"7\n";
           dist_W = (58*0.38)/(obj.w*0.00074);
          dist_H = (93*0.38)/(obj.h*0.00074);
          avg_dist = (dist_H + dist_W)/2.0;
          // a.depth_time = 5;


          break;
        case 8:
          std::cout<<"8\n";
           dist_W = (58*0.38)/(obj.w*0.00074);
          dist_H = (93*0.38)/(obj.h*0.00074);
          avg_dist = (dist_H + dist_W)/2.0;
          // a.depth_time = 5;

          break;
        }
      }
    }
  }catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
              image_frame->encoding.c_str());
    return;
  }
}

void task_stat(std_msgs::Int8::ConstPtr msg){
  tstate = 1;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "controller_test");
  
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  nh1 = new ros::NodeHandle(nh);
  
  ros::Publisher pub =
	      nh1->advertise<hammerhead_control::MoveCmd>("move_cmd", 100);
	  ros::Publisher pub1 = nh1->advertise<std_msgs::UInt8>("set_mode", 10, true);

  image_transport::Subscriber img_sub_fc = it.subscribe("/front_camera/image_rect_color", 1,
                            process_next_image_fc);

  std_msgs::UInt8 t;
  t.data = 2;

  pub1.publish(t);

  hammerhead_control::MoveCmd a;
  a.mode_after_last_cmd = 1;
  float int_yaw = std::stof(argv[1]);
  a.surge_speed = 0;
  a.surge_time = 0;
  a.depth_time = 5;
  a.depth_speed = 0.05;
  a.wait_for_max_timer_to_timeout = 1;
  a.depth = 0.45;
  a.isAbsolute = 0;
  a.is_depth = 1;
  a.is_yaw = 1;
  a.yaw = (int_yaw * 3.1415)/180.0;
  ros::Rate loop_rate(15);

  pub1.publish(t);

  ros::spinOnce();
  // ROS_INFO("Reached");
  ros::Time tt = ros::Time::now();
  while ((ros::Time::now() - tt).toSec() < 1)
    ;
  pub.publish(a);
  tt = ros::Time::now();

  while ((ros::Time::now() - tt).toSec() < 1)
    ;

  a.is_yaw = 1;
  a.yaw_time = 5;
  a.yaw = 0;
  a.is_depth = 0;
  a.is_surge = 0;
  a.is_sway = 0;
  pub.publish(a);
  

  pub.publish(a);
  // ROS_INFO("moved");

  a.surge_speed = 5;
  a.surge_time = 42;
  a.depth_time = 0;
  a.is_yaw = 0;
  a.is_depth = 0;
  a.is_surge = 1;
  pub.publish(a);

  a.surge_time = 0;
  a.yaw = 3.1415 * 4;
  a.yaw_time = 10;
  a.yaw_speed = 0.15;
  a.is_depth = 0;
  a.is_surge = 0;
  a.is_yaw = 1;
  pub.publish(a);


  a.surge_time = 0;
  a.yaw = -3.1415/9;
  a.yaw_time = 4;
  a.yaw_speed = -0.15;
  a.is_depth = 0;
  a.is_surge = 0;
  a.is_yaw = 1;
  pub.publish(a);

  a.surge_speed = 5;
  a.surge_time = 3;
  a.depth_time = 0;
  a.is_depth = 0;
  a.is_surge = 1;
  a.is_yaw = 0;
  a.mode_after_last_cmd = 1;
  pub.publish(a);

  a.surge_time = 0;
  a.yaw = 3.1415 * 0.25;
  a.yaw_time = 5;
  a.yaw_speed = 0.15;
  a.is_depth = 0;
  a.is_surge = 0;
  a.is_yaw = 1;
  pub.publish(a);

  a.surge_speed = 5;
  a.surge_time = 6;
  a.depth_time = 13;
  a.is_depth = 1;
  a.depth = 2;
  a.depth_speed = 0.1;
  a.is_surge = 1;
  a.is_yaw = 0;
  a.mode_after_last_cmd = 1;
  pub.publish(a);
ros::Rate l(15);
tstate = 0;
    ros::Subscriber s = nh.subscribe("/tasks_status", 1, task_stat);
    while(!tstate){ros::spinOnce();l.sleep();}

count = 0;

while(count < 10){ros::spinOnce();l.sleep();}

    avg_dist = avg_dist/1000;
    avg_sway = avg_sway/1000;
    avg_heave = avg_heave/1000;

    a.is_depth = 1;
    a.is_surge = 1;
    a.depth_speed = 0.1;
    a.depth = avg_heave;
    a.depth_time = a.depth/0.2;
    if (avg_sway>0)
    {a.is_sway = 1;
      a.sway_speed = 5;
      a.sway_time = avg_sway/0.185;
    }else if(avg_sway<0){
      a.is_sway = 1;
      a.sway_speed = -5;
      a.sway_time = avg_sway/0.194;
    }
    a.surge_speed=5;
    a.surge_time=(avg_dist/0.424)+3;
    pub.publish(a);

    a.is_depth = 0;
    a.sway_speed = 5;
    a.sway_time = 5;
    a.surge_speed = -5;
    a.surge_time -= 5;
    pub.publish(a);

    tstate = 0;
    while(!tstate){ros::spinOnce();l.sleep();}

    countd = 0;
    while(countd < 0){ros::spinOnce();l.sleep();}

    avg_dist_t = avg_dist_t/1000;
    avg_sway_t = avg_sway_t/1000;
    avg_heave_t = avg_heave_t/1000;

    a.is_depth = 1;
    a.is_surge = 1;
    a.depth_speed = 0.1;
    a.depth = avg_heave_t;
    a.depth_time = a.depth/0.2;
    if (avg_sway_t>0)
    {a.is_sway = 1;
      a.sway_speed = 5;
      a.sway_time = avg_sway_t/0.185;
    }else if(avg_sway_t<0){
      a.is_sway = 1;
      a.sway_speed = -5;
      a.sway_time = avg_sway_t/0.194;
    }
    a.surge_speed=5;
    a.surge_time=(avg_dist_t/0.424)+3;
    a.mode_after_last_cmd = 1;
    pub.publish(a);

   a.is_sway = 1;
   a.is_surge = 0;
  a.is_depth = 0;
  a.is_yaw = 0;
  a.sway = -5;
  a.sway_speed = -5;
 a.sway_time = 5;
  pub.publish(a);

 a.is_sway = 0;
 a.is_yaw = 1;
 a.yaw = -3.1415/6;
 a.yaw_speed = 0.05;
 a.yaw_time = 5;
 pub.publish(a);

a.is_yaw = 0;
a.is_surge = 1;
a.surge_speed = 5;
a.surge_time = 80;
a.mode_after_last_cmd = 0;
pub.publish(a);

  // ROS_INFO("moved");
  ros::spin();
  return 0;

}

