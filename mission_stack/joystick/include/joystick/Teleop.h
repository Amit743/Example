#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <pid_controller/Setpoint.h>
#include <pid_controller/PID.h>

class Teleop
{
public:
  Teleop(ros::NodeHandle _nh);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void setPoint_callback(const pid_controller::Setpoint::ConstPtr& msg);
  void PID_callback(const pid_controller::PID::ConstPtr& msg);
  ros::NodeHandle nh_;
  pid_controller::Setpoint s;
  pid_controller::PID p;
  ros::Publisher set_point_pub;
  ros::Publisher pid_pub;
  ros::Subscriber joy_sub_;
  ros::Subscriber set_point_sub;
  ros::Subscriber pid_sub;
};
