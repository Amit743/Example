#include "hammerhead_control/control.h"

Control::Control(ros::NodeHandle _nh) : nh(_nh) {

  std::string confFileName =
      ros::package::getPath("thruster_controller") + "/config/reverse.conf";
  std::ifstream confFile(confFileName.c_str());

  std::string confFileN =
      ros::package::getPath("pid_controller") + "/config/pid.conf";
  std::ifstream conf(confFileN.c_str());

  thruster_speeds =
      nh.advertise<thruster_controller::ThrusterSpeeds>("thruster_speeds", 10);

  pid_sub = nh.subscribe("/pid_params", 10, &Control::set_PID, this);

  mode_sub = nh.subscribe("/set_mode", 10, &Control::set_mode, this);

  move_cmd_sub = nh.subscribe("/move_cmd", 10, &Control::move_Cmd, this);

  move_cmds_sub = nh.subscribe("/move_cmds", 100, &Control::move_Cmds, this);

  thruster_status_pub =
      nh.advertise<std_msgs::Int8>("thruster_status", 10, true);

  position_pub =
      nh.advertise<hammerhead_control::Position>("bot_position", 10, true);

  tasks_status_pub = nh.advertise<std_msgs::Int8>("tasks_status", 1, true);

  tasks_status.data = 1;

  mode = SURFACE;

  timers_list = 0;

  kp = (float *)malloc(6 * sizeof(float));
  ki = (float *)malloc(6 * sizeof(float));
  kd = (float *)malloc(6 * sizeof(float));

  in = (float *)malloc(6 * sizeof(float));

  err = (float *)malloc(6 * sizeof(float));
  err_dot = (float *)malloc(6 * sizeof(float));
  perr = (float *)malloc(6 * sizeof(float));
  cerr = (float *)malloc(6 * sizeof(float));

  preSetPoint = (float *)malloc(6 * sizeof(float));
  preout = (float *)malloc(6 * sizeof(float));
  Forces = (float *)malloc(6 * sizeof(float));
  out = (float *)malloc(6 * sizeof(float));

  curr_set = (float *)malloc(6 * sizeof(float));
  goal_set = (float *)malloc(6 * sizeof(float));
  increment_set = (float *)malloc(6 * sizeof(float));
  move_time = (float *)malloc(6 * sizeof(float));

  int temp1;

  for (int i = 0; i < 6; i++) {
    F.data.push_back(1500);
    re_init.data.push_back(1500);
    if (i > 2) {
      conf >> kp[i - 3];
      conf >> ki[i - 3];
      conf >> kd[i - 3];
    } else {
      conf >> kp[i + 3];
      conf >> ki[i + 3];
      conf >> kd[i + 3];
    }

    confFile >> temp1;
    F.reverse.push_back(temp1);
    re_init.reverse.push_back(temp1);

    curr_set[i] = goal_set[i] = 0;
  }

  resetPID();

  for (int i = 0; i < 6; i++)
    AUV_DEBUG(kp[i], ki[i], kd[i]);

  thruster_status = 0;

  alpha[0] = 0.1;  // +2
  alpha[1] = 0.1;  // -2
  alpha[2] = 1.1;  // +3
  alpha[3] = 0.1;  // -3
  alpha[4] = 10.5; // +5
  alpha[5] = 0.3;  // -5
  alpha[6] = 1.3;  // +10
  alpha[7] = 10.5; // -10

  re_initialize_thrusters(2);
}

Control::~Control() {}

void Control::get_output(const synchronizer::Combined::ConstPtr &msg) {

  for (int i = 0; i < 3; i++) {
    in[i] = msg->angular[i] * (M_PI / 180.0);
    in[i + 3] = 0;
  }
  in[5] = msg->depth;
  in[1] = msg->angular[1] * M_PI / 180.0 - pitch_tare;
  in[0] = msg->angular[0] * M_PI / 180.0 - roll_tare;

  for (int i = 0; i < 3; i++) {
    err[i] = curr_set[i] - in[i];
    err[i] = atan2(sin(err[i]), cos(err[i]));
    err[i + 3] = curr_set[i + 3] - in[i + 3];
  }

  for (int i = 0; i < 6; i++) {
    err_dot[i] = err[i] - perr[i];
    perr[i] = err[i];
    cerr[i] += err[i]; // + or -
    if (cerr[i] > 2.55)
      cerr[i] = 2.55;
    else if (cerr[i] < -2.55)
      cerr[i] = -2.55;
    if (preSetPoint[i] != curr_set[i])
      cerr[i] = 0;
    preSetPoint[i] = curr_set[i];
  }

  for (int i = 0; i < 6; i++) {
    out[i] = kp[i] * err[i] + kd[i] * err_dot[i] + ki[i] * cerr[i];
    // out[i] = out[i] - 0.975 * (out[i] - preout[i]);
    // preout[i] = out[i];
  }
  if (curr_set[SURGE] == 0) {
    out[SURGE] =
        0.642 * err[SURGE] + 0.756 * err_dot[SURGE] + 0.007 * cerr[SURGE];
  }

  if (out[HEAVE] > 0.5)
    out[HEAVE] = out[HEAVE] - 0.975 * (out[HEAVE] - preout[HEAVE]);

  preout[5] = out[5];
  float alpha_tmp = 0;

  switch ((int)curr_set[1]) {
  case 2:
    alpha_tmp = alpha[0];
    break;
  case -2:
    alpha_tmp = alpha[1];
    break;
  case 3:
    alpha_tmp = alpha[2];
    break;
  case -3:
    alpha_tmp = alpha[3];
    break;
  case 5:
    alpha_tmp = alpha[4];
    break;
  case -5:
    alpha_tmp = alpha[5];
    break;
  case 10:
    alpha_tmp = alpha[6];
    break;
  case -10:
    alpha_tmp = alpha[7];
    break;
  default:
    alpha_tmp = 0;
  }

  Forces[0] = out[HEAVE] - out[PITCH] - out[ROLL];
  Forces[1] = out[HEAVE] - out[PITCH] + out[ROLL];
  Forces[2] = out[HEAVE] * 2 + 2 * out[PITCH];
  Forces[3] = out[YAW] + out[SURGE] + alpha_tmp * out[SWAY];
  Forces[4] = -out[YAW] + out[SURGE] - alpha_tmp * out[SWAY];
  Forces[5] = out[SWAY];

  /*  for (int i = 0; i < 6; i++) {
      AUV_DEBUG(curr_set[i], "\t", in[i], "\t", err[i], "\t");
    }

    AUV_DEBUG("\n");

    for (int i = 0; i < 6; i++)
      AUV_DEBUG(out[i], "\t", Forces[i]);
  */
  for (int i = 0; i < 6; i++) {

    if (Forces[i] >= tMin)
      Forces[i] = (Forces[i]) * 370 / 2.36 + 1530;

    else if (Forces[i] <= (-tMin))
      Forces[i] = 1470 + (Forces[i]) * 370 / 1.85;

    else
      Forces[i] = 1500;

    if (Forces[i] > 1750)
      Forces[i] = 1750;
    else if (Forces[i] < 1250)
      Forces[i] = 1250;
  }

  F.data[0] = (int16_t)Forces[0];
  F.data[1] = (int16_t)Forces[1];
  F.data[2] = (int16_t)Forces[2];
  F.data[3] = (int16_t)Forces[3];
  F.data[4] = (int16_t)Forces[4];
  F.data[5] = (int16_t)Forces[5];

  thruster_stat.data = 1;
  thruster_status_pub.publish(thruster_stat);
  thruster_speeds.publish(F);

  if (curr_set[SURGE] > 0)

    {
pos.x += (ros::WallTime::now() - combined_callback_timer).toSec() *
             forward_surge * cos(msg->angular[2] * 3.141 / 180.0);
pos.y += (ros::WallTime::now() - combined_callback_timer).toSec() *
              forward_surge * sin(msg->angular[2] * 3.141 / 180.0);
}
  else if (curr_set[SURGE] < 0)
{
    pos.x += (ros::WallTime::now() - combined_callback_timer).toSec() *
             backward_surge * cos(msg->angular[2] * 3.141 / 180.0);
    pos.y += (ros::WallTime::now() - combined_callback_timer).toSec() *
              backward_surge * sin(msg->angular[2] * 3.141 / 180.0);
}
  if (curr_set[SWAY] < 0)
{
    pos.y += (ros::WallTime::now() - combined_callback_timer).toSec() *
              left_sway * cos(msg->angular[2] * 3.141 / 180.0);
    pos.x += (ros::WallTime::now() - combined_callback_timer).toSec() *
              left_sway * sin(msg->angular[2] * 3.141 / 180.0);
}
  else if (curr_set[SWAY] > 0)
    {pos.y += (ros::WallTime::now() - combined_callback_timer).toSec() *
              right_sway * cos(msg->angular[2] * 3.141 / 180.0);
	pos.x += (ros::WallTime::now() - combined_callback_timer).toSec() *
              right_sway * sin(msg->angular[2] * 3.141 / 180.0);
}
combined_callback_timer = ros::WallTime::now();

  pos.z = msg->depth;
  pos.roll = 0;
  pos.pitch = 0;
  pos.yaw = msg->angular[2];

  position_pub.publish(pos);
}

void Control::re_initialize_thrusters(uint8_t a) {

  thruster_stat.data = a;
  thruster_status_pub.publish(thruster_stat);
  sleep(0.5);
  AUV_DEBUG("Called reinit", (int)a);
  thruster_speeds.publish(re_init);

}

void Control::resetPID() {

  for (int i = 0; i < 6; i++)
    preout[i] = in[i] = err[i] = err_dot[i] = perr[i] = cerr[i] = Forces[i] =
        out[i] = preSetPoint[i] = 0;
}

void Control::move_Cmd(const hammerhead_control::MoveCmd &msg) {

  ROS_INFO("Move c m d received", move_count);

  move_count++;

  ROS_INFO("Move c md received", move_count);

  switch (mode) {

  case SURFACE:
    return;
  }

  tasks_list.push(msg);

  AUV_DEBUG("Move cmd received", move_count, (int)tasks_list.size());

  if (tasks_list.size() == 1) {
    start_next_cmd();
  }
}

void Control::start_next_cmd() {

  AUV_DEBUG("Check", curr_move_cmd, (int)tasks_list.size());
  if (tasks_list.empty()) {

    std_msgs::UInt8 m;
    m.data = mode_after_last_cmd;

    set_mode(m);

    return;
  }

  curr_move_cmd++;
  AUV_DEBUG("Starting next cmd", curr_move_cmd, (int)tasks_list.size());

  hammerhead_control::MoveCmd next = tasks_list.front();

  wait_for_max_timer_to_timeout = next.wait_for_max_timer_to_timeout;
  isAbsolute = next.isAbsolute;
  mode_after_last_cmd = next.mode_after_last_cmd;

  move_time[ROLL] = 0;
  move_time[PITCH] = 0;
  goal_set[ROLL] = 0;
  goal_set[PITCH] = 0;

  if (next.is_yaw) {
    move_time[YAW] = next.yaw_time;
    increment_set[YAW] = next.yaw_speed;

    if (isAbsolute)
      goal_set[YAW] = next.yaw;

    else
      goal_set[YAW] = curr_set[YAW] + next.yaw;

    load_next_command(YAW);
  }

  if (next.is_sway) {
    move_time[SWAY] = next.sway_time;
    increment_set[SWAY] = next.sway_speed;
    goal_set[SWAY] = next.sway_speed;
    load_next_command(SWAY);
  }

  else
    curr_set[SWAY] = 0;

  if (next.is_surge) {
    move_time[SURGE] = next.surge_time;
    increment_set[SURGE] = next.surge_speed;
    goal_set[SURGE] = next.surge_speed;
    load_next_command(SURGE);
  } else
    curr_set[SURGE] = 0;
  if (next.is_depth) {

    move_time[HEAVE] = next.depth_time;
    increment_set[HEAVE] = next.depth_speed;

    if (isAbsolute)
      goal_set[HEAVE] = next.depth;

    else
      goal_set[HEAVE] = curr_set[HEAVE] + next.depth;

    load_next_command(HEAVE);
  }

  if (next.is_yaw && increment_set[YAW] != 0 && move_time[YAW] != 0) {

    AUV_DEBUG("Yaw");
    yaw_start_time = ros::WallTime::now();
    yaw_timer =
        nh.createTimer(ros::Duration(0.5), &Control::yaw_timer_callback, this);
    timers_list = timers_list | 0x01;
  }

  if (next.is_surge && increment_set[SURGE] != 0 && move_time[SURGE] != 0) {
    AUV_DEBUG("Surge");
    surge_start_time = ros::WallTime::now();
    surge_timer =
        nh.createTimer(ros::Duration(1), &Control::surge_timer_callback, this);
    timers_list = timers_list | 0x02;
  }

  if (next.is_sway && increment_set[SWAY] != 0 && move_time[SWAY] != 0) {
    AUV_DEBUG("Sway");
    sway_start_time = ros::WallTime::now();
    sway_timer =
        nh.createTimer(ros::Duration(1), &Control::sway_timer_callback, this);
    timers_list = timers_list | 0x04;
  }

  if (next.is_depth && increment_set[HEAVE] != 0 && move_time[HEAVE] != 0) {
    AUV_DEBUG("Depth");
    depth_start_time = ros::WallTime::now();
    depth_timer = nh.createTimer(ros::Duration(0.5),
                                 &Control::depth_timer_callback, this);
    timers_list = timers_list | 0x08;
  }

  std_msgs::UInt8 m;
  m.data = MOVEMENT;
  set_mode(m);
}

void Control::load_next_command(int i) {

  if (goal_set[i] > goal_upper_limit[i])
    goal_set[i] = goal_upper_limit[i];

  if (goal_set[i] < goal_lower_limit[i])
    goal_set[i] = goal_lower_limit[i];

  if (increment_set[i] > speed_upper_limit[i])
    increment_set[i] = speed_upper_limit[i];

  if (increment_set[i] < speed_lower_limit[i])
    increment_set[i] = speed_lower_limit[i];

  if (move_time[i] > time_limit[i])
    move_time[i] = time_limit[i];
}

void Control::yaw_timer_callback(const ros::TimerEvent &event) {
  AUV_DEBUG("YawCallback");

  bool tmp = increment_setpoint(YAW);

  if (tmp ||
      (ros::WallTime::now() - yaw_start_time).toSec() >= move_time[YAW]) {
    yaw_timer.stop();
    stop_timer(0x01);
  }
}

void Control::surge_timer_callback(const ros::TimerEvent &event) {

  AUV_DEBUG("SurgeCallback");
  bool tmp = increment_setpoint(SURGE);

  if (tmp ||
      (ros::WallTime::now() - surge_start_time).toSec() >= move_time[SURGE]) {
    surge_timer.stop();
    stop_timer(0x02);
  }
}

void Control::sway_timer_callback(const ros::TimerEvent &event) {
  AUV_DEBUG("SwayCallback");

  bool tmp = increment_setpoint(SWAY);
  if (tmp ||
      (ros::WallTime::now() - sway_start_time).toSec() >= move_time[SWAY]) {
    sway_timer.stop();
    stop_timer(0x04);
  }
}

void Control::depth_timer_callback(const ros::TimerEvent &event) {
  AUV_DEBUG("DepthCallback");

  bool tmp = increment_setpoint(HEAVE);

  if (tmp ||
      (ros::WallTime::now() - depth_start_time).toSec() >= move_time[HEAVE]) {
    depth_timer.stop();
    stop_timer(0x08);
  }
}

void Control::stop_timer(uint8_t a) {

  if (!(timers_list & a))
    return;
  timers_list = timers_list & (~a);

  AUV_DEBUG("stop_timer ", (int)a);

  if (!timers_list) {
    tasks_list.pop();
    start_next_cmd();
  }
}

void Control::stop_and_clear_tasks() {

  depth_timer.stop();
  surge_timer.stop();
  sway_timer.stop();
  yaw_timer.stop();

  timers_list = 0;
  tasks_list = std::queue<hammerhead_control::MoveCmd>();
}

bool Control::increment_setpoint(int a) {

  curr_set[a] += increment_set[a];
  AUV_DEBUG("incr ", goal_set[a], "\t", curr_set[a]);
  if(increment_set[a] > 0){
  if (goal_set[a] - curr_set[a] <= 0) {
    curr_set[a] = goal_set[a];
    if (!wait_for_max_timer_to_timeout)
      return 1;
  }}
else if(increment_set[a] < 0){
  if (goal_set[a] - curr_set[a] >= 0) {
    curr_set[a] = goal_set[a];
    if (!wait_for_max_timer_to_timeout)
      return 1;
  }}
  return 0;
}

void Control::set_mode(const std_msgs::UInt8 &msg) {

  AUV_DEBUG("set mode callback", (int)mode, (int)msg.data);

  if (mode == msg.data)
    return;

  switch (msg.data) {

  case SURFACE:

    stop_and_clear_tasks();

    pos.x = 0;
    pos.y = 0;

    if (thruster_status) {
      disable_output();
      thruster_status = 0;
    }

    resetPID();
    re_initialize_thrusters(2);

    break;

  case HOVER:

    curr_set[ROLL] = 0;
    curr_set[PITCH] = 0;
    curr_set[SURGE] = 0;
    curr_set[SWAY] = 0;

    stop_and_clear_tasks();
    if (mode == SURFACE) {
      re_initialize_thrusters(2);
      resetPID();
      sleep(0.25);
      curr_set[HEAVE] = surface_depth;
      if (!thruster_status) {
        enable_output();
        thruster_status = 1;
      }
    }

    tasks_status_pub.publish(tasks_status);

    break;

  case MOVEMENT:

    if (mode == SURFACE) {
      memset(curr_set, 0, sizeof(float) * 5);
      re_initialize_thrusters(2);
      resetPID();
      sleep(0.25);
      curr_set[HEAVE] = surface_depth;
      if (!thruster_status) {
        enable_output();
	combined_callback_timer = ros::WallTime::now();
        thruster_status = 1;
      }
    }

    break;

  default:
    stop_and_clear_tasks();
    if (thruster_status) {
      disable_output();
      thruster_status = 0;
    }
    resetPID();
    re_initialize_thrusters(2);
    mode = SURFACE;
    return;
  }

  mode = msg.data;
}

void Control::move_Cmds(const hammerhead_control::MoveCmds::ConstPtr &msg) {

  for (int i = 0; i < msg->len; i++) {
    tasks_list.push(msg->list[i]);
  }
}

void Control::disable_output() {

  if (in_sub.getNumPublishers() != 0) {
    in_sub.shutdown();
  }
}

void Control::enable_output() {

  if (in_sub.getNumPublishers() == 0)
    in_sub = nh.subscribe("/combined", 1, &Control::get_output, this);
}

void Control::set_PID(const pid_controller::PID::ConstPtr &msg) {
  for (int i = 0; i < 3; i++) {
    kp[i + 3] = msg->kp[i];
    ki[i + 3] = msg->ki[i];
    kd[i + 3] = msg->kd[i];
    kp[i] = msg->kp[i + 3];
    ki[i] = msg->ki[i + 3];
    kd[i] = msg->kd[i + 3];
  }
}
