#include <iostream>
#include <chrono>

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::system_clock::time_point TimePoint;

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <hammerhead/hammerhead.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <pid_controller/Setpoint.h>
#include <vectornav/VectorNavData.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include "path_planner/FindGate.h"
#include "path_planner/GateDetector.h"
#include "path_planner/YawControl.h"
#include "path_planner/AlignBin.h"
#include "path_planner/BinsTask.h"

using namespace cv;
using namespace std;

/**

 * \brief an enum type for tasks

 */
enum task {NONE,DROP,RELEASEPICKER,LIFTPICKER,PICK};

/**

 * \brief the below class all the variables and functions for planning the path to perform all the tasks

*/
class PathPlanner{
public:

	/**

	 * \brief parameterised constructor ; creates all tasks related objects , initializes all subscribers and publishers , bool parameters , etc.

	 * SUBSCRIBED TOPICS : /front_camera/image_rect_color  /bottom_camera/image_rect_color  /vectornav

   * PUBLISHED TOPICS : /front_camera_debug  /bottom_camera/debug  /setpoints  /actuator/ball_drop
	                      /breach_command  /actuator/picker_ascend  /actuator/picker_descend

	 * \param[in] _nh ros NodeHandle

	 * \param[in] int(1) stores next sway mode

	 * \param[in] int(2) stores sway range

	 */
	explicit PathPlanner(ros::NodeHandle _nh, int, int);

/**
 destructor
 */
	~PathPlanner();

/**

  * \brief function publishes setpoints (output values) till the gate is detected

	*/
    void publishSetpoints();

/**

  * \brief function finds required setpoints for detecting the gate

	*/
    void findSetpoints();

  /**

	 * \brief function receives the front camera feed of the auv , identifies all tasks related objects and debugs it accordingly ;
	          publishes through frontDebugPub

	*/
    void frontCameraCallback(const sensor_msgs::ImageConstPtr& msg);

	/**

		* \brief function receives the bottom camera feed of the auv , identifies all tasks related objects and debugs it accordingly ;
 	          publishes through bottomDebugPub

	*/
    void bottomCameraCallback(const sensor_msgs::ImageConstPtr& msg);

	/**

	 * \brief function receives vectornav data to get the current yaw value and stores it

	 */
    void vectornavCallback(const vectornav::VectorNavData::ConstPtr &msg);

		/**

	   * \brief function for finding out time difference between auv moving commands for finding blue bin and returns the same

	   * \param[in] TimePoint(1) current clock time

	   * \param[in] TimePoint(2) start time of task

	   */
    double findTimeDifference(TimePoint , TimePoint );

		/**

		 * \brief function controls the ball drop mechanism

		 */
    void ballDropSequence();

		/**

			* \brief function controls the ball pick mechanism

		*/
    void ballPickupSequence();

    ros::NodeHandle nh;
    image_transport::ImageTransport it; //!< image transport handle

    image_transport::Publisher frontDebugPub, bottomDebugPub;

    ros::Publisher setpointPub, drop_pub, breachPub, descend_pub, ascend_pub;
    ros::Subscriber frontCameraSub, bottomCameraSub, vectornavSub;

    pid_controller::Setpoint sp; //!< Setpoint type message
    sensor_msgs::ImagePtr frontDebugMsg, bottomDebugMsg;
    std_msgs::Int8 descendMsg, ascendMsg;
    std_msgs::Int8 dropMsg;
    std_msgs::Bool breachMsg;

    FindGate *findGateOb;
    GateDetector *gateDetectorOb;
    YawControl *yawControlOb;
    AlignBin *alignBinOb;
    BinsTask *binsTaskOb;

    bool USE_YAW_CORRECTION = false, pauseYawCorrection;
    const int MAX_FIND_GATE_FRAMES = 50;
    const int CORRECT_YAW_FRAMES = 75;
    const int GATE_CROSS_TIME = 30;
    const int FORWARD_SEARCH_TIME = 10;
    const int BASE_YAW_SEARCH_ANGLE = 15;

    int gateFrameCount;

    TimePoint start_1,start_2;

    bool gateSearchStarted, gateFound, gateAligned, gateReached;
    bool gateFinalYawAligned, gateFinalAligned, gateCrossed;
    bool returnToCenter;
    bool foundBin, reachedBin, alignedWithBin, binTaskComplete;
    bool gateDetectedLast;
    bool goingForwardToSearch = true;
    int goingForwardCount = 0;

    const int imageWidth = 648, imageHeight = 488;
    const double imgArea = imageWidth * imageHeight;

    double currentYaw, headingDiff, storedYaw, yawChange = 0;
    int yawChangeDir = 1;
    int yawJump = 3;

    TimePoint startTime, lastFrontCameraCallbackTime, gateCrossTimer;
    TimePoint swayCheckTime, swayReturnTimer;
    TimePoint prevCommandTime;
    TimePoint lastYawCorrectTime;
    TimePoint heightChangedLast;
    TimePoint yawChangedLast;
    TimePoint forwardSearch;

    Vec3i binsSetpoint;
    bool binTaskTimeout;
    int bin_not_found_ctr = 0;
    double rightSwayUnits;
    vector<task> tasks; //!< stores tasks list
    vector<state> stateList; //!< stores the state list
    vector<int> stateTimeouts; //!< stores state timeout for each state execution
};
