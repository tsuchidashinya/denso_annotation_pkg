#pragma once
#include <util/util.hpp>
#include <common_msgs/CloudData.h>
#include <common_msgs/Translation.h>
#include <common_msgs/Rotation.h>


class NoizeCloudTransform
{
public:
    static common_msgs::CloudData translation_noize(common_msgs::CloudData, geometry_msgs::Vector3);
};