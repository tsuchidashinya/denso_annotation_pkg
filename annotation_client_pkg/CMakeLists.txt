cmake_minimum_required(VERSION 3.0.2)
project(annotation_client_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  util
  gazebo_model_package
  tf_package
  annotation_common
  labeling_package
  data_transform_pkg
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES annotation_client_pkg
#  CATKIN_DEPENDS anno_msgs anno_srvs common_msgs common_srvs cv_bridge pcl_ros roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(semseg src/segmentation/semantic_segmentation_client.cpp src/annotation_client_node.cpp)
target_link_libraries(semseg
  ${catkin_LIBRARIES}
)
add_executable(semseg_randomizer src/segmentation/semantic_segmentation_randomizer_client.cpp src/annotation_client_node.cpp)
target_link_libraries(semseg_randomizer
  ${catkin_LIBRARIES}
)
add_executable(object_detection_2d src/object_detection/object_detection_2d_client.cpp src/annotation_client_node.cpp)
target_link_libraries(object_detection_2d
  ${catkin_LIBRARIES}
)
add_executable(acc_annotation src/acc_annotation.cpp src/annotation_client_node.cpp)
target_link_libraries(acc_annotation
  ${catkin_LIBRARIES}
)
add_executable(real_cloud_dataset_get src/real_data/real_cloud_dataset_get.cpp)
target_link_libraries(real_cloud_dataset_get
  ${catkin_LIBRARIES}
)

add_executable(real_acc_annotation src/real_data/real_acc_annotation.cpp src/annotation_client_node.cpp)
target_link_libraries(real_acc_annotation
  ${catkin_LIBRARIES}
)