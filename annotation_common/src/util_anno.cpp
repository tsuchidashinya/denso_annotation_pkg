#include <annotation_common/util_anno.hpp>

std::vector<std::string> UtilAnno::tf_listen_frames_from_objectinfo(std::vector<common_msgs::ObjectInfo> objectinfos)
{
    std::vector<std::string> outdata;
    outdata.resize(objectinfos.size());
    for (int i = 0; i < outdata.size(); i++) {
        outdata[i] = objectinfos[i].tf_name;
    }
    return outdata;
}