#include "hammerhead_control/control.h"

Control *obj;
bool sint = false;

void sig_handler(int signum) {
  obj->re_initialize_thrusters(0);
  sint = true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "control_node");
  ros::NodeHandle nh;

  signal(SIGINT, sig_handler);
  signal(SIGTERM, sig_handler);

  obj = new Control(nh);

  ros::Rate loop_rate(100);

  while (ros::ok() && !sint) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
