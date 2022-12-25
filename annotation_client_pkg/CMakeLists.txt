cmake_minimum_required(VERSION 3.0.2)
project(annotation_client_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  util
  gazebo_model_package
  tf_package
  
  labeling_package
  data_transform_pkg
  sensor_package
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES annotation_client_pkg
#  CATKIN_DEPENDS common_msgs common_srvs cv_bridge pcl_ros roscpp
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
add_executable(object_detection_multi_object src/object_detection/object_detection_multi_object_client.cpp src/annotation_client_node.cpp)
target_link_libraries(object_detection_multi_object
  ${catkin_LIBRARIES}
)
add_executable(acc_annotation src/acc/acc_annotation.cpp)
target_link_libraries(acc_annotation
  ${catkin_LIBRARIES}
)

add_executable(real_acc_annotation src/acc/real_acc_annotation.cpp src/annotation_client_node.cpp)
target_link_libraries(real_acc_annotation
  ${catkin_LIBRARIES}
)

add_executable(real_acc_annotation_from_hdf5_acc src/acc/real_acc_annotation_from_hdf5_acc.cpp src/annotation_client_node.cpp src/annotation_client_func.cpp)
target_link_libraries(real_acc_annotation_from_hdf5_acc
  ${catkin_LIBRARIES}
)

add_executable(real_acc_annotation_from_hdf5_sensor src/acc/real_acc_annotation_from_hdf5_sensor.cpp src/annotation_client_node.cpp src/annotation_client_func.cpp)
target_link_libraries(real_acc_annotation_from_hdf5_sensor
  ${catkin_LIBRARIES}
)
add_executable(semantic_segmentation_multi_object_client src/segmentation/semantic_segmentation_multi_object_client.cpp src/annotation_client_node.cpp)
target_link_libraries(semantic_segmentation_multi_object_client
  ${catkin_LIBRARIES}
)

add_executable(semantic_segmentation_multi_object_kai_client src/segmentation/semantic_segmentation_multi_object_kai_client.cpp src/annotation_client_node.cpp)
target_link_libraries(semantic_segmentation_multi_object_kai_client
  ${catkin_LIBRARIES}
)

add_executable(real_cloud_dataset_get src/real_data/real_cloud_dataset_get.cpp)
target_link_libraries(real_cloud_dataset_get
  ${catkin_LIBRARIES}
)

add_executable(semantic_segmentation_no_b_box src/segmentation/semantic_segmentation_no_b_box.cpp src/annotation_client_node.cpp)
target_link_libraries(semantic_segmentation_no_b_box
  ${catkin_LIBRARIES}
)

add_executable(semantic_segmentation_no_sensor src/segmentation/semantic_segmentation_no_sensor.cpp src/annotation_client_node.cpp)
target_link_libraries(semantic_segmentation_no_sensor
  ${catkin_LIBRARIES}
)

add_executable(semantic_segmentation_sensor_bbox src/segmentation/semantic_segmentation_sensor_bbox.cpp src/annotation_client_node.cpp)
target_link_libraries(semantic_segmentation_sensor_bbox
  ${catkin_LIBRARIES}
)