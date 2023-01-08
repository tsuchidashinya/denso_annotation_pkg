#pragma once
#include <util/util.hpp>
#include <common_msgs/CloudData.h>
#include <math.h>
#include <space_handling_pkg/space_handling_library.hpp>
#include "noize_cloud_transform.hpp"

class NoizeCloudMake
{
public:
    common_msgs::CloudData sphere_cloud(float, int);
    common_msgs::CloudData sphere_empty_cloud(float, int);
    common_msgs::CloudData circle_cloud(float, float, float, int);
    common_msgs::CloudData linear_cloud(float, float, int);
    common_msgs::CloudData cylinder_cloud(float, float, int);
    common_msgs::CloudData rectangle_cloud(float, float, float, int);
    common_msgs::CloudData noize_cloud_random();
    common_msgs::CloudData noize_cloud_random(float, float, float);
    static common_msgs::CloudData make_defect_cloud(common_msgs::CloudData, common_msgs::CloudData, double);
    void set_parameter();
private:
    Util util_;
    XmlRpc::XmlRpcValue param_list;
};