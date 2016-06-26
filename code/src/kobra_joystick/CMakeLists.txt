cmake_minimum_required(VERSION 2.8.3)
project(kobra_joystick)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(joystick src/kobra_joystick.cpp)
target_link_libraries(joystick ${catkin_LIBRARIES} )

catkin_package()

#############
## Install ##
#############

#############
## Testing ##
#############
