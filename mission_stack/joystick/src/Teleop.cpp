#include <joystick/Teleop.h>

Teleop::Teleop(ros::NodeHandle _nh)
{
  ros::param::set("joy_node/dev", "/dev/input/js0" );
  set_point_pub = nh_.advertise<pid_controller::Setpoint>("setpoints", 1);
  set_point_sub = nh_.subscribe<pid_controller::Setpoint>("/setpoints", 1, &Teleop::setPoint_callback, this);
  pid_sub = nh_.subscribe<pid_controller::PID>("/pid_params", 1, &Teleop::PID_callback, this);
  pid_pub = nh_.advertise<pid_controller::PID>("pid_params", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &Teleop::joyCallback, this);
  for(int i = 0; i < 6; i++){
    s.setpoints.push_back(0);
    p.kp.push_back(0);
    p.ki.push_back(0);
    p.kd.push_back(0);
  }
s.setpoints[5]=980;
}

void Teleop::PID_callback(const pid_controller::PID::ConstPtr& msg){
    for(int i = 0; i < 6;i++)
    {
        p.kp[i] = msg->kp[i];
        p.ki[i] = msg->ki[i];
        p.kd[i] = msg->kd[i];
    }
}

void Teleop::setPoint_callback(const pid_controller::Setpoint::ConstPtr& msg)
{
    for(int i = 0; i < 6;i++)
    s.setpoints[i] = msg->setpoints[i];
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std::cout << "Axes\n";
  for(int i = 0;i < 6; i++)
  std::cout << joy->axes[i] << "\n";
  std::cout << "Buttons\n";
  for(int i = 0;i < 12; i++)
  std::cout << joy->buttons[i] << "\n";

  if(joy->axes[0] > 0.5)
    s.setpoints[5] -= 10;
  else
    if(joy->axes[0] < -0.5)
        s.setpoints[5] += 10;

  if(joy->axes[3] > 0.5)
    s.setpoints[5]++;
  else
    if(joy->axes[3] < -0.5)
        s.setpoints[5]--;


  s.setpoints[0] += joy->buttons[0];
  s.setpoints[0] -= joy->buttons[2];

  s.setpoints[1] += joy->buttons[1];
  s.setpoints[1] -= joy->buttons[3];

  s.setpoints[2] += 10 * joy->buttons[7];
  s.setpoints[2] -= 10 * joy->buttons[5];

  s.setpoints[4] += 10 * joy->buttons[4];
  s.setpoints[4] -= 10 * joy->buttons[6];

  if(s.setpoints[0] > 5) s.setpoints[0] = 5;
  else if(s.setpoints[0] < -5) s.setpoints[0] = -5;

  if(s.setpoints[1] > 5) s.setpoints[1] = 5;
  else if(s.setpoints[1] < -5) s.setpoints[1] = -5;

  if(s.setpoints[2] > 1075) s.setpoints[2] = 1075;
  else if(s.setpoints[2] < 980) s.setpoints[2] = 980;

  if(s.setpoints[3] > 180) s.setpoints[3] = 180;
  else if(s.setpoints[3] < -180) s.setpoints[3] = 180;

  if(s.setpoints[4] > 180) s.setpoints[4] = 180;
  else if(s.setpoints[4] < -180) s.setpoints[4] = 180;

  if(s.setpoints[5] > 180) s.setpoints[5] = 180;
  else if(s.setpoints[5] < -180) s.setpoints[5] = 180;

  if(joy->buttons[9])
  {
    s.setpoints[0] = s.setpoints[1] = 0;
  }

  if(joy->buttons[8])
  {
    s.setpoints[0] = s.setpoints[1] = s.setpoints[3] = s.setpoints[4]  = s.setpoints[5] = 0;
    s.setpoints[2] = 980;
  }
  set_point_pub.publish(s);
  pid_pub.publish(p);
}
