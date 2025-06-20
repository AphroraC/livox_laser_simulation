cmake_minimum_required(VERSION 3.15.0)
project(livox_laser_simulation)

## Compile as C++17, supported in ROS Noetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  livox_ros_driver
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

#To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/gazebo-7/gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})

#This is designed for whose proto installed in system is not 2.6. We can install the version of proto in local dir
#include_directories(/home/lfc/proto/include/)
#link_directories(/home/lfc/proto/lib/)

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        livox_ros_driver
        livox_laser_simulation
        CATKIN_DEPENDS
        tf
)

add_library(livox_laser_simulation_pointcloud SHARED
        src/livox_points_plugin.cpp
        src/livox_ode_multiray_shape.cpp
        src/plugin_pointcloud.cpp
        )
target_link_libraries(livox_laser_simulation_pointcloud ${catkin_LIBRARIES} RayPlugin)
target_link_libraries(livox_laser_simulation_pointcloud libprotobuf.so.9)

add_library(livox_laser_simulation_pointcloud2 SHARED
        src/livox_points_plugin.cpp
        src/livox_ode_multiray_shape.cpp
        src/plugin_pointcloud2.cpp
        )
target_link_libraries(livox_laser_simulation_pointcloud2 ${catkin_LIBRARIES} RayPlugin)
target_link_libraries(livox_laser_simulation_pointcloud2 libprotobuf.so.9)

add_library(livox_laser_simulation_livoxcustom SHARED
        src/livox_points_plugin.cpp
        src/livox_ode_multiray_shape.cpp
        src/plugin_livoxcustom.cpp
        )
target_link_libraries(livox_laser_simulation_livoxcustom ${catkin_LIBRARIES} RayPlugin)
target_link_libraries(livox_laser_simulation_livoxcustom libprotobuf.so.9)