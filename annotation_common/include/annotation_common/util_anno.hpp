#pragma once
#include <util/util_base.hpp>
#include <anno_msgs/ObjectInfo.h>

class UtilAnno
{
public:
    static std::vector<std::string> get_tf_frames_from_objectinfo(std::vector<anno_msgs::ObjectInfo>);
};