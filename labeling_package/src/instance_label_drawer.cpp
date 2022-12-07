#include <labeling_package/instance_label_drawer.hpp>

common_msgs::CloudData InstanceLabelDrawer::draw_instance_all(common_msgs::CloudData cloud, int instance)
{
    cloud.instance.resize(cloud.x.size());
    for (int i = 0; i < cloud.x.size(); i++) {
        cloud.instance[i] = instance;
    }
    return cloud;
}

InstanceLabelDrawer::InstanceLabelDrawer() : pnh_("~")
{
    set_parameter();
}

void InstanceLabelDrawer::set_parameter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
}

common_msgs::CloudData InstanceLabelDrawer::extract_nearest_point(common_msgs::CloudData sensor_cloud, common_msgs::CloudData mesh_cloud, int instance, double radious = 0.004)
{
    pcl::PointCloud<PclXyz> sensor_pcl, mesh_pcl;
    sensor_pcl = UtilMsgData::cloudmsg_to_pcl(sensor_cloud);
    mesh_pcl = UtilMsgData::cloudmsg_to_pcl(mesh_cloud);
    pcl::search::KdTree<PclXyz> kdtree;
    kdtree.setInputCloud(sensor_pcl.makeShared());
    std::vector<int> pointIndices;
    std::vector<float> squaredDistance;
    ros::WallTime start = ros::WallTime::now();
    for (auto mesh : mesh_pcl.points) {
        if (kdtree.radiusSearch(mesh, radious, pointIndices, squaredDistance)) {
            for (int j = 0; j < pointIndices.size(); j++) {
                ros::WallTime end = ros::WallTime::now();
                ros::WallDuration calc_time = end - start;
                if (calc_time.toSec() >= 2) {
                    ROS_WARN_STREAM("nearest point search failed");
                    return sensor_cloud;
                }
                else {
                    sensor_cloud.instance[pointIndices[j]] = instance;
                }
            }
        }
        pointIndices.clear();
        squaredDistance.clear();
    }
    return sensor_cloud;
}



/*
1: cloud_multi std::vector<common_msgs::CloudData>
2: occuluder_instance int
3: occludy_instance int
4: radious double
*/
std::vector<common_msgs::CloudData> InstanceLabelDrawer::extract_occuluder(std::vector<common_msgs::CloudData> cloud_multi, double radious)
{
    std::vector<common_msgs::CloudData> out_data;
    std::vector<std::string> tf_names;
    for (int i = 0; i < cloud_multi.size(); i++) {
        if (cloud_multi[i].tf_name.size() != 0) {
            tf_names.push_back(cloud_multi[i].tf_name);
        }
    }
    std::vector<ObjectTfNameType> occluder_list = detect_occuluder(tf_names, radious);
    for (int i = 0; i < cloud_multi.size(); i++) {
        for (int j = 0; j < occluder_list.size(); j++) {
            if (i == occluder_list[j].index) {
                out_data.push_back(cloud_multi[i]);
            }
        }
    }
    return out_data;
}

std::vector<common_msgs::ObjectInfo> InstanceLabelDrawer::extract_occuluder(std::vector<common_msgs::ObjectInfo> object_info, double radious)
{
    std::vector<common_msgs::ObjectInfo> out_data;
    std::vector<std::string> tf_names;
    for (int i = 0; i < object_info.size(); i++) {
        tf_names.push_back(object_info[i].tf_name);
    }
    std::vector<ObjectTfNameType> occluder_list = detect_occuluder(tf_names, radious);
    for (int i = 0; i < object_info.size(); i++) {
        for (int j = 0; j < occluder_list.size(); j++) {
            if (i == occluder_list[j].index) {
                out_data.push_back(object_info[i]);
            }
        }
    }
    return out_data;
}

std::vector<ObjectTfNameType> InstanceLabelDrawer::detect_occuluder(std::vector<std::string> tf_names, double radious)
{
    std::vector<ObjectTfNameType> occluder_list;
    int collision = 0;
    for (int i = 0; i < tf_names.size(); i++) {
       geometry_msgs::Transform trans_get;
       
       trans_get = tf_func_.tf_listen(tf_names[i], world_frame_);
    //    TfFunction::tf_data_show(trans_get, "tf_name_" + std::to_string(i));
       if (i == 0) {
        ObjectTfNameType object;
        object.trans = trans_get;
        object.index = i;
        occluder_list.push_back(object);
       }
       else {
        collision = 0;
        std::vector<int> delete_list;
        for (int k = 0; k < occluder_list.size(); k++) {
            double dis = Util::distance(trans_get.translation.x, trans_get.translation.y, occluder_list[k].trans.translation.x, occluder_list[k].trans.translation.y);
            if (dis < 2 * radious) {
                if (trans_get.translation.z > occluder_list[k].trans.translation.z) {
                    // occluder_list.erase(occluder_list.begin() + k);
                    delete_list.push_back(occluder_list[k].index);
                }
                else {
                    collision = 1;
                    break;
                }
            }
        }
        // std::cout << "occuluder_list: ";
        // for (int i = 0; i < occluder_list.size(); i++) {
        //        std::cout << occluder_list[i].index << " "; 
        // }
        // std::cout << std::endl;
        // std::cout << "delete_list: ";
        for (int i = 0; i < delete_list.size(); i++) {
            // std::cout << delete_list[i] << " ";
            for (int j = 0; j < occluder_list.size(); j++) {
                int index = 0;
                if (occluder_list[j].index == delete_list[i]) {
                    occluder_list.erase(occluder_list.begin() + j);
                    break;
                }
            }
        }
        // std::cout << std::endl;
        
        if (collision == 0) {
            ObjectTfNameType object;
            object.trans = trans_get;
            object.index = i;
            occluder_list.push_back(object);
        }
       } 
    }
    return occluder_list;
}

std::vector<common_msgs::BoxPosition> InstanceLabelDrawer::set_object_class_name(std::vector<common_msgs::BoxPosition> box_position_list, std::string class_name)
{
    for (int i = 0; i < box_position_list.size(); i++) {
        box_position_list[i].object_name = class_name;
    }
    return box_position_list;
}