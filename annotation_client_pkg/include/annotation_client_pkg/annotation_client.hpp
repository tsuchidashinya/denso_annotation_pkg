#pragma once
#include <common_srvs/MeshCloudService.h>
#include <sensor_package/cloud_process.hpp>
// #include <common_srvs/RecordService.h>
#include <common_srvs/Hdf5RecordRealSensorData.h>
#include <common_srvs/SensorService.h>
#include <common_srvs/VisualizeImage.h>
#include <common_srvs/VisualizeCloud.h>
#include <common_srvs/TfBroadcastService.h>
#include <common_srvs/VisualizeCloudDelete.h>
#include <common_srvs/Hdf5RecordSegmentation.h>
#include <common_srvs/Hdf5RecordAcc.h>
#include <common_srvs/Hdf5OpenService.h>
#include <common_srvs/Hdf5OpenRealPhoxiService.h>
#include <gazebo_model_package/decide_object_position.hpp>
#include <gazebo_model_package/gazebo_model_move.hpp>
#include <tf_package/tf_function.hpp>
#include <util/util.hpp>
#include <labeling_package/instance_label_drawer.hpp>
#include <data_transform_pkg/data_3D_to_2D.hpp>
#include <data_transform_pkg/data_2D_to_3D.hpp>
#include <data_transform_pkg/func_data_convertion.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/Empty.h>

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
    ros::ServiceClient sensor_client_, mesh_client_, visualize_client_, hdf5_record_client_, gazebo_sensor_client_, 
            vis_img_client_, tf_br_client_, vis_delete_client_, hdf5_open_client_;
    std::string sensor_service_name_, mesh_service_name_, visualize_service_name_, hdf5_record_service_name_, 
            vis_img_service_name_, tf_br_service_name_, vis_delete_service_name_, hdf5_open_service_name_;
    std::string gazebo_sensor_service_name_;
    std::string world_frame_, sensor_frame_;
    std::string save_dir_, save_base_file_name_;
    std::vector<std::string> object_list_;
    std::vector<int> quantity_of_object_list_;
    std::vector<int> instance_of_object_list_;
    TfFunction tf_func_;
    UtilMsgData util_msg_data_;
    Util util_;
    InstanceLabelDrawer instance_drawer_;
    CloudProcess cloud_process_;
    ros::Publisher domain_randomize_pub_;
    double q_x_para_, q_y_para_, q_z_para_, x_para_, y_para_, z_para_;
    double xyz_step_, qxyz_step_;
    double nearest_radious_, occlusion_object_radious_;
    std::string hdf5_record_file_path_, hdf5_open_file_path_;
};