cmake_minimum_required(VERSION 3.0.2)
project(annotation_common)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  common_srvs
  util
)


catkin_python_setup()


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES util_anno
 CATKIN_DEPENDS common_srvs util
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


