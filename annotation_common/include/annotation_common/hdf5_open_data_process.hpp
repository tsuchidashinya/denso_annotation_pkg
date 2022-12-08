#pragma once
#include <util/util.hpp>
#include <util/util_msg_data.hpp>
#include <common_srvs/VisualizeCloud.h>
#include <common_srvs/VisualizeSensorPC2.h>
#include <common_srvs/Hdf5OpenService.h>
#include <common_srvs/RecordAcc.h>
#include <common_srvs/RecordSegmentation.h>
#include <tf_package/tf_function.hpp>
#include <opencv2/opencv.hpp>

class Hdf5OpenDataProcess
{
public:
    Hdf5OpenDataProcess(ros::NodeHandle&);
    void main();
    void acc_main(int);
    void set_parameter();
    XmlRpc::XmlRpcValue param_list;
    int the_number_of_execute_;

private:
    ros::NodeHandle nh_, pnh_;
    ros::ServiceClient sensor_client_, object_detect_client_, visualize_client_, cloud_network_client_, accuracy_client_, 
    hdf5_open_client_, vis_image_client_, hdf5_record_client_;
    std::string sensor_service_name_, object_detect_service_name_, visualize_service_name_,
    cloud_network_service_name_, accuracy_service_name_, hdf5_open_service_name_, vis_image_service_name_, hdf5_record_service_name_;
    std::string world_frame_, sensor_frame_;
    TfFunction tf_func_;
    UtilMsgData util_msg_data_;
};