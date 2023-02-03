#pragma once
#include <util_package/util.hpp>
#include <common_msgs/CloudData.h>
#include <math.h>
#include <space_handling_pkg/space_handling_library.hpp>
#include "noize_cloud_transform.hpp"
#include <sensor_package/cloud_process.hpp>

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
    common_msgs::CloudData noize_tube_small_z_range(float, float);
    common_msgs::CloudData noize_tube_small();
    common_msgs::CloudData noize_tube_big();
    static common_msgs::CloudData make_defect_cloud(common_msgs::CloudData, common_msgs::CloudData, double);
    void set_parameter();
private:
    Util util_;
    XmlRpc::XmlRpcValue param_list;
};