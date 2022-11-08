#include <util_anno/util_anno.hpp>

std::vector<std::string> UtilAnno::get_tf_frames_from_objectinfo(std::vector<anno_msgs::ObjectInfo> objectinfos)
{
    std::vector<std::string> outdata;
    outdata.resize(objectinfos.size());
    for (int i = 0; i < outdata.size(); i++) {
        outdata[i] = objectinfos[i].tf_name;
    }
    return outdata;
}