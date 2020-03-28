cmake_minimum_required(VERSION 2.8.3)
project(hammerhead_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  roslib
  pid_controller
  hammerhead
  std_msgs
  message_generation
  openvino_object_detection
  thruster_controller
  synchronizer
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  MoveCmd.msg
  MoveCmds.msg
  Position.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})
set(H include/hammerhead_control/control.h)
SET(SOURCES src/control.cpp)

add_library(control ${SOURCES} ${H})
add_executable(thruster_speed src/main.cpp)

target_link_libraries(thruster_speed control ${catkin_LIBRARIES})
add_dependencies(control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(control_test src/test.cpp)

target_link_libraries(control_test control  ${OpenCV_LIBS} ${catkin_LIBRARIES} )
add_dependencies(control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

