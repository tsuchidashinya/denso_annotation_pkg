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

void UtilAnno::write_b_box_label(std::vector<common_msgs::BoxPosition> boxes, std::string save_file_path)
{
    std::ofstream file(save_file_path);
    for (int i = 0; i < boxes.size(); i++) {
        file << boxes[i].object_class_name;
        file << " " << boxes[i].x_one << " " << boxes[i].y_one;
        file << " " << boxes[i].x_two << " " << boxes[i].y_two;
        file << std::endl;
    }
}