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
  space_handling_pkg
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES noize_cloud_make noize_cloud_transform noize_cloud_client noize_image_client
#  CATKIN_DEPENDS roscpp util
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(noize_cloud_make src/cloud/noize_cloud_make.cpp)
target_link_libraries(noize_cloud_make
  ${catkin_LIBRARIES}
)

add_library(noize_cloud_transform src/cloud/noize_cloud_transform.cpp)
target_link_libraries(noize_cloud_transform
  ${catkin_LIBRARIES}
)

add_library(noize_image_make src/image/noize_image_make.cpp)
target_link_libraries(noize_image_make
  ${catkin_LIBRARIES}
)

add_library(noize_cloud_client src/cloud/noize_cloud_client.cpp)
target_link_libraries(noize_cloud_client
  noize_cloud_make
  noize_cloud_transform
  ${catkin_LIBRARIES}
)

add_library(noize_image_client src/image/noize_image_client.cpp)
target_link_libraries(noize_image_client
  noize_image_make
  ${catkin_LIBRARIES}
)

add_executable(noize_image_client_node src/image/noize_image_client_node.cpp)
target_link_libraries(noize_image_client_node
  noize_image_client
  ${catkin_LIBRARIES}
)

add_executable(noize_cloud_client_node src/cloud/noize_cloud_client_node.cpp)
target_link_libraries(noize_cloud_client_node
  noize_cloud_client
  ${catkin_LIBRARIES}
)

add_executable(noize_cloud_try src/cloud/noize_cloud_try.cpp src/cloud/noize_cloud_client_node.cpp)
target_link_libraries(noize_cloud_try
  noize_cloud_make
  noize_cloud_transform
  ${catkin_LIBRARIES}
)