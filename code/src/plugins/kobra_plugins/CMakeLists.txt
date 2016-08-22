cmake_minimum_required(VERSION 2.8.3)
project(kobra_plugins)


# Depend on system install of Gazebo
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(tf REQUIRED)
find_package(kobra_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${kobra_msgs_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(pantilt_camera SHARED src/pantilt_camera.cpp)
target_link_libraries(pantilt_camera ${GAZEBO_libraries} ${roscpp_LIBRARIES})

add_library(steering_control SHARED src/steering_control.cpp)
target_link_libraries(steering_control ${GAZEBO_libraries} ${roscpp_LIBRARIES})

add_library(kobra_tf SHARED src/kobra_transformation_frame.cpp)
target_link_libraries(kobra_tf ${GAZEBO_libraries} ${roscpp_LIBRARIES} ${tf_LIBRARIES})
