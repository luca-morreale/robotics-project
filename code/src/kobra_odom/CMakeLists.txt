cmake_minimum_required(VERSION 2.8.3)
project(kobra_odom)

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

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(kobra_pose_node src/pose_node.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(kobra_pose_node ${catkin_LIBRARIES})

catkin_package()

#############
## Install ##
#############

#############
## Testing ##
#############
