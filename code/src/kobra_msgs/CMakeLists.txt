cmake_minimum_required(VERSION 2.8.3)
project(kobra_msgs)

find_package(catkin REQUIRED COMPONENTS  
  roscpp  
  gazebo_ros
  std_msgs 
  message_generation 
) 

link_directories(${GAZEBO_LIBRARY_DIRS}) 
include_directories(include 
  ${Boost_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS} 
  ${OGRE_INCLUDE_DIRS} 
  ${OGRE-Terrain_INCLUDE_DIRS} 
  ${OGRE-Paging_INCLUDE_DIRS} 
) 

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files( 
  FILES 
  ptz_msg.msg 
) 

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

generate_messages( 
  DEPENDENCIES 
  std_msgs 
)

catkin_package( 
  DEPENDS  
    roscpp  
    gazebo_ros 
    CATKIN_DEPENDS message_runtime 
) 


#############
## Install ##
#############

#############
## Testing ##
#############
