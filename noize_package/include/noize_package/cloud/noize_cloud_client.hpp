#pragma once
#include <util/util.hpp>
#include <util/util_msg_data.hpp>
#include <common_srvs/VisualizeCloud.h>
#include <common_srvs/VisualizeImage.h>
#include <common_srvs/Hdf5OpenAccService.h>
#include <common_srvs/Hdf5OpenSegmentationService.h>
#include <common_srvs/Hdf5RecordAcc.h>
#include <common_srvs/Hdf5RecordSegmentation.h>
#include <tf_package/tf_function.hpp>
#include <opencv2/opencv.hpp>
#include <noize_package/cloud/noize_cloud_make.hpp>
#include <noize_package/cloud/noize_cloud_transform.hpp>
#include <noize_package/image/noize_image_make.hpp>
#include <sensor_package/cloud_process.hpp>

class NoizeCloudClient
{
public:
    NoizeCloudClient(ros::NodeHandle&);
    void main();
    void acc_main(int);
    void set_parameter();
    XmlRpc::XmlRpcValue param_list;
    int the_number_of_execute_;

private:
    ros::NodeHandle nh_, pnh_;
    ros::ServiceClient sensor_client_, object_detect_client_, visualize_client_, cloud_network_client_, accuracy_client_, 
    hdf5_open_client_, vis_image_client_, hdf5_hdf5_record_client_;
    std::string sensor_service_name_, object_detect_service_name_, visualize_service_name_,
    cloud_network_service_name_, accuracy_service_name_, hdf5_open_acc_service_name_, vis_image_service_name_, hdf5_hdf5_record_service_name_;
    std::string world_frame_, sensor_frame_;
    std::string hdf5_open_file_path_;
    TfFunction tf_func_;
    UtilMsgData util_msg_data_;
    Util util_;
};