#pragma once
#include <util/util.hpp>
#include <anno_msgs/ObjectInfo.h>

class UtilAnno
{
public:
    static std::vector<std::string> tf_listen_frames_from_objectinfo(std::vector<anno_msgs::ObjectInfo>);
};