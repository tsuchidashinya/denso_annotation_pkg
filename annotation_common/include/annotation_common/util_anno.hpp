#pragma once
#include <util/util.hpp>
#include <common_msgs/ObjectInfo.h>

class UtilAnno
{
public:
    static std::vector<std::string> tf_listen_frames_from_objectinfo(std::vector<common_msgs::ObjectInfo>);
};