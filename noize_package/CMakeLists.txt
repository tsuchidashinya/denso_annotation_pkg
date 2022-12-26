cmake_minimum_required(VERSION 3.0.2)
project(noize_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  util
  common_srvs
  tf_package
  sensor_package
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES noize_cloud_make noize_cloud_transform
#  CATKIN_DEPENDS roscpp util
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(noize_cloud_make src/noize_cloud_make.cpp)
target_link_libraries(noize_cloud_make
  ${catkin_LIBRARIES}
)

add_library(noize_cloud_transform src/noize_cloud_transform.cpp)
target_link_libraries(noize_cloud_transform
  ${catkin_LIBRARIES}
)

add_executable(noize_client src/noize_client.cpp src/noize_client_node.cpp)
target_link_libraries(noize_client
  noize_cloud_make
  noize_cloud_transform
  ${catkin_LIBRARIES}
)