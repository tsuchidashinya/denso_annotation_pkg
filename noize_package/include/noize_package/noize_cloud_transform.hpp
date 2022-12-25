#pragma once
#include <util/util.hpp>
#include <common_msgs/CloudData.h>
#include <common_msgs/Translation.h>
#include <common_msgs/Rotation.h>
#include <tf_package/tf_function.hpp>


class NoizeCloudTransform
{
public:
    static common_msgs::CloudData translation_noize(common_msgs::CloudData, geometry_msgs::Vector3);
    static common_msgs::CloudData rotate_noize(common_msgs::CloudData, geometry_msgs::Quaternion);
};