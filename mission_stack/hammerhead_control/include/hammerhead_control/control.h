#include <fstream>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>

#include <hammerhead/hammerhead.h>
#include <hammerhead_control/MoveCmd.h>
#include <hammerhead_control/MoveCmds.h>
#include <hammerhead_control/Position.h>
#include <queue>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <synchronizer/Combined.h>
#include <thruster_controller/ThrusterSpeeds.h>

#include <pid_controller/PID.h>

#include <csignal>

/**

 * \brief Contains all members , variables , functions required for controlling the auv. 

*/
class Control {
public:

/**

 * \brief Parameterised constructor for Control type objects.
 
 * \param[in] _nh ros nodehandle for handling the control node.
 
 * SUBSCRIBED TOPICS : /pid_params   /set_mode   /move_cmd   /move_cmds  
 
 * PUBLISHED TOPICS : /thruster_status   bot_position   tasks_status

*/
  explicit Control(ros::NodeHandle _nh);

/** 

 * destructror for Control type objects.

*/
  ~Control();

/** 

 * \brief Function to reinitialize thrusters.

 * \param[in] an unsigned 8-bit integer for mode of the thruster as per enum Mode.

 * Publishes thruster_stat and re_init (Thruster Speed type) messages over thruster_status_pub
   and thruster_speeds publishers respectively.

*/   
  void re_initialize_thrusters(uint8_t a);

private:

/**

 * \brief An enum type for mode of the auv. 

*/
  enum Mode { SURFACE, HOVER, MOVEMENT };

/**

 * \brief An enum type for degrees of freedom of the auv.

*/

  enum DOF { ROLL, PITCH, YAW, SURGE, SWAY, HEAVE };

  ros::NodeHandle nh; //!< a ros nodehandle 
  ros::Subscriber in_sub; //!< subscriber for topic /combined 
  ros::Subscriber mode_sub; //!< subscriber for topic /set_mode
  ros::Subscriber move_cmd_sub; //!< subscriber for topic /move_cmd
  ros::Subscriber move_cmds_sub; //!< subscriber for topic /move_cmds
  ros::Subscriber pid_sub; //!< subscriber for topic /pid_params

  ros::Publisher position_pub; //!< publisher for topic bot_position
  ros::Publisher tasks_status_pub; //!< publisher for topic tasks_status
  ros::Publisher thruster_speeds; //!< publisher for topic thruster_speeds
  ros::Publisher thruster_status_pub; //!< publisher for topic thruster_status

  ros::Timer depth_timer; //!< timer for managing the time between consecutive heave commands 
  ros::Timer yaw_timer; //!< timer for managing the time between consecutive yaw commands
  ros::Timer surge_timer; //!< timer for managing the time between consecutive surge commands
  ros::Timer sway_timer; //!< timer for managing the time between consecutive sway commands

  uint8_t timers_list; //!< an unsigned integer for storing the hexadecimal equivalent of time in seconds

  std_msgs::Int8 thruster_stat; //!< message for thruster status
  std_msgs::Int8 tasks_status; //!< message for tasks status

  std::queue<hammerhead_control::MoveCmd> tasks_list; //!< queue of MoveCmd message type for performing the tasks

  hammerhead_control::MoveCmd current; //!< MoveCmd type message for current move command

  hammerhead_control::Position pos; //!< Position type message for x , y , z coordinates and angular accelerations

  ros::WallTime combined_callback_timer; 

  float forward_surge = 0.424; //!< default forward surge
  float backward_surge = -0.458; //!< default backward surge
  float left_sway = -0.194; //!< default left sway
  float right_sway = 0.185; //!< default right sway

  int move_count = 0; //!< stores no. of move commands received
  int curr_move_cmd = 0; //!< counter for the command no. being executed

  float *kp, *ki, *kd, *in, *err, *err_dot, *perr, *cerr, *out, *preout,
      *preSetPoint; //!< pointers for pid controller use

  const float tMax = 2.36;
  const float tMin = 0.01;
  const float tRF = (tMax - tMin) / pow((1520 - 1480), 2);
  const float tRB = tRF * 0.7;
  const float pitch_tare = 0;
  const float roll_tare = 8.5;
  const float surface_depth = 0.22;

  float alpha[8];

  float *Forces; //!< pointer for storing forces for the thrusters

  thruster_controller::ThrusterSpeeds F; //!< Thruster Speed type message storing current thruster speed and directn of rotation
  thruster_controller::ThrusterSpeeds re_init; //!< Thruster Speed type message storing default thruster speed and directn of rotation

  const float goal_lower_limit[6] = {0, 0, -180, -5, -5, surface_depth}; //!< lower limit goals for each thruster
  const float goal_upper_limit[6] = {0, 0, 180, 5, 5, 10}; //!< upper limit goals for each thruster

  const float speed_lower_limit[6] = {0, 0, -30, -5, -5, -1}; //!< lower speed limit goals for each thruster
  const float speed_upper_limit[6] = {0, 0, 30, 5, 5, 1}; //!< upper speed limit goals for each thruster

  const float time_limit[6] = {0, 0, 100, 40, 40, 4000}; //!< default time limit for executing the command
  float *increment_set; //!< pointer storing increment setpoints for each thrusters
  float *curr_set; //!< pointer storing current setpoints for each thrusters
  float *goal_set; //!< pointer storing goal setpoints for each thrusters
  float *move_time; //!< pointer storing time for executing command for each thrusters

  ros::WallTime yaw_start_time;
  ros::WallTime surge_start_time;
  ros::WallTime sway_start_time;
  ros::WallTime depth_start_time;

  bool thruster_status = 0; //!< 0 : thrusters not in use , 1 : thrusters in use
  bool wait_for_max_timer_to_timeout;
  bool isAbsolute;
  uint8_t mode_after_last_cmd; //!< stores last command mode
  uint8_t mode; //!< stores current mode

/** 

 * \brief provides output for moving the thrusters

 * \param[in] msg a pointer of Combined type message

*/ 
  void get_output(const synchronizer::Combined::ConstPtr &msg);

/**
 
 * \brief gets the pid values

 * \param[in] msg a pointer of PID type message

*/
  void set_PID(const pid_controller::PID::ConstPtr &msg);

/**

 * \brief receives the move command and pushes to the tasks_lists queue

 * \param[in] msg a pointer of MoveCmd type message

*/  
  void move_Cmd(const hammerhead_control::MoveCmd &msg);

/**

 * \brief recieves a bunch of move commands together and pushes to tasks_lists queue

 * \param[in] msg a pointer of MoveCmds type message 

*/
  void move_Cmds(const hammerhead_control::MoveCmds::ConstPtr &msg);

/**
 
 * \brief updates the current set points

 * \param[in] a integer denoting the thruster being accessed

*/
  bool increment_setpoint(int a);

/**

 * \brief updates the mode of the auv

 * \param[in] msg a pointer of unsigned 8 bit-integer

*/
  void set_mode(const std_msgs::UInt8 &msg);

/**
 
 * \brief loads next command 

 * \param[in] i integer for denoting the thruster no.

*/
  void load_next_command(int i);

/**

 * \brief callback function for depth_timer initialisation

 * \param[in] event a pointer for Timer object changes

*/
  void depth_timer_callback(const ros::TimerEvent &event);

/**

 * \brief callback function for yaw_timer initialisation

 * \param[in] event a pointer for Timer object changes

*/
  void yaw_timer_callback(const ros::TimerEvent &event);

/**

 * \brief callback function for surge_timer initialisation

 * \param[in] event a pointer for Timer object changes

*/
  void surge_timer_callback(const ros::TimerEvent &event);

/**

 * \brief callback function for sway_timer initialisation

 * \param[in] event a pointer for Timer object changes

*/
  void sway_timer_callback(const ros::TimerEvent &event);

/**

 * \brief function for executing next commands

*/
  void start_next_cmd();

/**

 * \brief function for stopping timers

 * \param[in] an unsigned 8-bit integer for time in seconds for stopping 

*/
  void stop_timer(uint8_t a);

/**

 * \brief function for disabling output

*/
  void disable_output();

/**

 * \brief function for enabling output

 * SUBSRIBED TOPIC : /combined

*/
  void enable_output();

/**

 * \brief function for resetting pid values

*/
  void resetPID();

/**

 * \brief function for clearing the queue of tasks_lists 

*/
  void stop_and_clear_tasks();
};
