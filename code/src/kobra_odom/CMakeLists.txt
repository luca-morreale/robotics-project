cmake_minimum_required(VERSION 2.8.3)
project(kobra_odom)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
)

catkin_package()

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(kobra_joystick
#   src/${PROJECT_NAME}/kobra_joystick.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(kobra_joystick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(kobra_pose_node src/kobra_pose_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kobra_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} src/kobra_pose_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(kobra_pose_node ${catkin_LIBRARIES})
