#pragma once
#include <common_srvs/MeshCloudService.h>
#include <sensor_package/cloud_process.hpp>
// #include <common_srvs/RecordService.h>
#include <common_srvs/Hdf5RecordSensorData.h>
#include <common_srvs/SensorService.h>
#include <common_srvs/VisualizeImage.h>
#include <common_srvs/VisualizeCloud.h>
#include <common_srvs/TfBroadcastService.h>
#include <common_srvs/TfDeleteService.h>
#include <common_srvs/VisualizeDeleteService.h>
#include <common_srvs/Hdf5RecordSegmentation.h>
#include <common_srvs/Hdf5RecordAcc.h>
#include <common_srvs/Hdf5OpenAccService.h>
#include <common_srvs/Hdf5OpenSensorDataService.h>
#include <gazebo_model_pkg/decide_object_position.hpp>
#include <gazebo_model_pkg/gazebo_model_move.hpp>
#include <tf_package/tf_function.hpp>
#include <util_package/util.hpp>
#include <util_package/common_header.hpp>
#include <space_handling_pkg/space_handling_library.hpp>
#include <data_transform_pkg/data_3D_to_2D.hpp>
#include <data_transform_pkg/data_2D_to_3D.hpp>
#include <data_transform_pkg/func_data_convertion.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/Empty.h>
#include <noize_package/image/noize_image_client.hpp>



class AnnotationClient
{
public:
    AnnotationClient(ros::NodeHandle &);
    bool main();
    void set_paramenter();
    common_msgs::CloudData nearest_extractor(common_msgs::CloudData, common_srvs::MeshCloudService, int);
    common_srvs::MeshCloudService mesh_request(std::string);
    void visualize_request(common_msgs::CloudData, std::string);
    void tf_broadcast_request(geometry_msgs::TransformStamped);
    common_msgs::CloudData crop_cloudmsg(common_msgs::CloudData);
    XmlRpc::XmlRpcValue param_list;
    int the_number_of_dataset_;
    int counter_;

private:
    ros::NodeHandle nh_, pnh_;
    ros::ServiceClient sensor_client_, mesh_client_, visualize_client_, hdf5_record_client_, gazebo_sensor_client_, 
            vis_img_client_, tf_br_client_, vis_delete_client_, hdf5_open_client_, tf_delete_client_, hdf5_record_2_client_;
    std::string sensor_service_name_, mesh_service_name_, visualize_service_name_, hdf5_record_service_name_, hdf5_record_2_service_name_,
            vis_img_service_name_, tf_br_service_name_, vis_delete_service_name_, hdf5_open_acc_service_name_, tf_delete_service_name_;
    std::string gazebo_sensor_service_name_;
    std::string world_frame_, sensor_frame_;
    std::string save_dir_, save_base_file_name_;
    std::vector<ObjectListType> object_option_list_;
    TfFunction tf_func_;
    UtilMsgData util_msg_data_;
    Util util_;
    SpaceHandlingLibrary instance_drawer_;
    CloudProcess cloud_process_;
    double q_x_para_, q_y_para_, q_z_para_, x_para_, y_para_, z_para_;
    double xyz_step_, qxyz_step_;
    double nearest_radious_;
    std::string hdf5_record_file_path_, hdf5_open_file_path_;
    DecidePosition decide_gazebo_object_;
};