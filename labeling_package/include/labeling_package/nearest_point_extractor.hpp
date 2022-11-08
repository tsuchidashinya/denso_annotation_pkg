#pragma once
#include <common_msgs/CloudData.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <util/util_sensor.hpp>

class NearestPointExtractor
{
public:
    NearestPointExtractor();
    static common_msgs::CloudData draw_initial_instance(common_msgs::CloudData, int);
    static common_msgs::CloudData extract_nearest_point(common_msgs::CloudData, common_msgs::CloudData, int, double);
};