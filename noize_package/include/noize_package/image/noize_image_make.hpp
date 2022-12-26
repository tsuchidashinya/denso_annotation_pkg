#pragma once
#include <util/util.hpp>
#include <common_msgs/CloudData.h>
#include <sensor_msgs/Image.h>

class NoizeImageMake
{
public:
    NoizeImageMake();
    void set_parameter();
    cv::Mat plane_image(int, int, int);
private:
    ros::NodeHandle pnh_;
    XmlRpc::XmlRpcValue param_list;
};