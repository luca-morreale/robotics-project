cmake_minimum_required(VERSION 2.8.3)
project(kobra_plugins)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  std_msgs
  tf
  message_generation
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

add_message_files(
  FILES
  ptz_msg.msg
)

add_library(pantilt_camera src/pantilt_camera.cpp)
target_link_libraries(pantilt_camera ${catkin_LIBRARIES} ${GAZEBO_libraries})

add_library(steering_control src/steering_control.cpp)
target_link_libraries(steering_control ${catkin_LIBRARIES} ${GAZEBO_libraries})

add_library(kobra_tf src/kobra_transformation_frame.cpp)
target_link_libraries(kobra_tf ${catkin_LIBRARIES} ${GAZEBO_libraries})

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

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
