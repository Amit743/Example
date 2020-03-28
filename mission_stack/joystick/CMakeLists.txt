cmake_minimum_required(VERSION 2.8.3)
project(joystick)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  hammerhead
  sensor_msgs
  pid_controller
)

set(H include/joystick/Teleop.h)
SET(SOURCES src/Teleop.cpp)


catkin_package()
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_library(joystick ${SOURCES} ${H})
target_link_libraries(joystick ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(joy_teleop src/main.cpp)
target_link_libraries(joy_teleop joystick)
