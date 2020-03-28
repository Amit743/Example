#include <path_planner/PathPlanner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;

    bool swayRight = argv[1][0] - '0';
    int  swayRange = argv[2][0] - '0';
    int nextSwayMode;
    if(swayRight)
        nextSwayMode = 1;
    else
        nextSwayMode = -1;
    PathPlanner obj(nh,nextSwayMode,swayRange);
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
    	ros::spinOnce();
    	loop_rate.sleep();
        obj.findSetpoints();
        obj.publishSetpoints();
    }

    return 0;
}
