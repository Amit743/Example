cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
hammerhead
roscpp
roslib
std_msgs
sensor_msgs
image_transport
genmsg
message_filters
message_generation
cv_bridge
pid_controller
vectornav
)

find_package(OpenCV 3.2 REQUIRED)

catkin_package()

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(FindGate include/path_planner/FindGate.h src/FindGate.cpp)

add_library(YawControl include/path_planner/YawControl.h src/YawControl.cpp)
target_link_libraries(YawControl ${OpenCV_LIBS})

add_library(GateDetector include/path_planner/GateDetector.h src/GateDetector.cpp)
target_link_libraries(GateDetector ${OpenCV_LIBS})

add_library(AlignBin include/path_planner/AlignBin.h src/AlignBin.cpp)
target_link_libraries(AlignBin ${OpenCV_LIBS})

add_library(BinsTask include/path_planner/BinsTask.h src/BinsTask.cpp)
target_link_libraries(BinsTask ${OpenCV_LIBS})

add_library(PathPlanner include/path_planner/PathPlanner.h src/PathPlanner.cpp)
target_link_libraries(PathPlanner GateDetector YawControl FindGate AlignBin BinsTask ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(path_planner src/main.cpp)
target_link_libraries(path_planner PathPlanner)

## TASK 1
add_executable(qualification_task src/qualification_task.cpp)
target_link_libraries(qualification_task ${catkin_LIBRARIES} ${OpenCV_LIBS})
