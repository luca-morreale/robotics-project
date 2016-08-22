cmake_minimum_required(VERSION 2.8.3)
project(simple_navigation_goals)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
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

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

catkin_package()

#############
## Install ##
#############

#############
## Testing ##
#############
