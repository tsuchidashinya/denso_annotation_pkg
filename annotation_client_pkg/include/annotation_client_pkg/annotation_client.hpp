#pragma once
#include <anno_srvs/MeshCloudService.h>
// #include <anno_srvs/RecordService.h>
#include <anno_srvs/RecordRealSensorData.h>
#include <common_srvs/SensorService.h>
#include <common_srvs/VisualizeImage.h>
#include <common_srvs/VisualizeCloud.h>
#include <common_srvs/GazeboSensorMoveService.h>
#include <anno_srvs/RecordSegmentation.h>
#include <anno_srvs/RecordAcc.h>
#include <gazebo_model_package/decide_object_position.hpp>
#include <gazebo_model_package/gazebo_model_move.hpp>
#include <tf_package/tf_function.hpp>
#include <util/util.hpp>
#include <labeling_package/instance_label_drawer.hpp>
#include <data_transform_pkg/make_2Dinfo_by_3D.hpp>
#include <data_transform_pkg/get_3D_by_2D.hpp>
#include <data_transform_pkg/func_data_convertion.hpp>
#include <annotation_common/util_anno.hpp>
#include <opencv2/opencv.hpp>

class AnnotationClient
{
public:
    AnnotationClient(ros::NodeHandle &);
    void main();
    void set_paramenter();
    XmlRpc::XmlRpcValue param_list;
    int the_number_of_dataset_;

private:
    ros::NodeHandle nh_, pnh_;
    ros::ServiceClient sensor_client_, mesh_client_, visualize_client_, record_client_, gazebo_sensor_client_, vis_img_client_;
    std::string sensor_service_name_, mesh_service_name_, visualize_service_name_, record_service_name_, vis_img_service_name_;
    std::string gazebo_sensor_service_name_;
    std::string world_frame_, sensor_frame_;
    std::string save_dir_, save_base_file_name_;
    std::string object_name_;
    TfFunction tf_func_;
    UtilMsgData util_msg_data_;
    Util util_;
    InstanceLabelDrawer instance_drawer_;
    double q_x_para_, q_y_para_, q_z_para_, x_para_, y_para_, z_para_;
    double xyz_step_, qxyz_step_;
    double nearest_radious_;
};