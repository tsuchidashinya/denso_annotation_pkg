#pragma once
#include <util/util.hpp>
#include <common_msgs/CloudData.h>
#include <math.h>

class NoizeCloudMake
{
public:
    common_msgs::CloudData sphere_unit(float, int);
    common_msgs::CloudData cylinder_unit(float, float, int);
    common_msgs::CloudData sphere_multi();
    void set_parameter();
private:
    Util util_;
    XmlRpc::XmlRpcValue param_list;
};