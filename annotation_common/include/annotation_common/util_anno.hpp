#pragma once
#include <util/util.hpp>
#include <common_msgs/ObjectInfo.h>
#include <common_msgs/BoxPosition.h>
#include <filesystem>
#include <fstream>

class UtilAnno
{
public:
    static std::vector<std::string> tf_listen_frames_from_objectinfo(std::vector<common_msgs::ObjectInfo>);
    static void write_b_box_label(std::vector<common_msgs::BoxPosition>, std::string);
};