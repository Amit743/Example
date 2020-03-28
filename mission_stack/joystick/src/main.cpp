#include <joystick/Teleop.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_hammerhead");
  ros::NodeHandle _nh;
  Teleop teleop_auv(_nh);

  ros::spin();
}
