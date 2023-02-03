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

class NoizeImageClient
{
public:
    NoizeImageClient(ros::NodeHandle&);
    void noize_image_main();
    void noize_image_main(std::string);
    void restore_image();
    void set_parameter();
    XmlRpc::XmlRpcValue param_list;
    int the_number_of_execute_;

private:
    ros::NodeHandle nh_, pnh_;
    ros::ServiceClient vis_image_client_;
    std::string vis_image_service_name_;
    std::string world_frame_, sensor_frame_;
    TfFunction tf_func_;
    UtilMsgData util_msg_data_;
    Util util_;
    int small_max_count_, big_max_count_;
};