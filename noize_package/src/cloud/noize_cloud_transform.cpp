#include <noize_package/cloud/noize_cloud_transform.hpp>


common_msgs::CloudData NoizeCloudTransform::change_frame_id(common_msgs::CloudData cloud, std::string frame_before, std::string frame_after) {
    TfFunction tf_func_;
    geometry_msgs::Transform trans_ori, trans_add;
    auto quat_zero = TfFunction::rotate_xyz_make(0, 0, 0);
    trans_ori.rotation = TfFunction::tf2_quat_to_geo_quat(quat_zero);
   
    trans_add = tf_func_.tf_listen(frame_before, frame_after);
    // TfFunction::tf_data_show(trans_add, "trans_add_cloud");
        
    common_msgs::CloudData out_cloud;
    for (int i = 0; i < cloud.x.size(); i++) {
        trans_ori.translation.x = cloud.x[i];
        trans_ori.translation.y = cloud.y[i];
        trans_ori.translation.z = cloud.z[i];
        auto trans_after = TfFunction::change_tf_frame_by_rotate(trans_ori, trans_add);
        out_cloud.x.push_back(trans_after.translation.x);
        out_cloud.y.push_back(trans_after.translation.y);
        out_cloud.z.push_back(trans_after.translation.z);
        out_cloud.instance.push_back(cloud.instance[i]);
    }
    out_cloud = UtilMsgData::substitute_cloudmsg_para(out_cloud, cloud);
    return out_cloud;
}

geometry_msgs::Vector3 NoizeCloudTransform::change_frame_id(geometry_msgs::Vector3 vector3, std::string frame_before, std::string frame_after) {
    TfFunction tf_func_;
    geometry_msgs::Transform trans_ori;
    trans_ori.translation = vector3;
    auto quat_zero = TfFunction::rotate_xyz_make(0, 0, 0);
    trans_ori.rotation = TfFunction::tf2_quat_to_geo_quat(quat_zero);
    auto trans_add = tf_func_.tf_listen(frame_before, frame_after);
    // TfFunction::tf_data_show(trans_add, "trans_add_vector3");
    auto trans_after = TfFunction::change_tf_frame_by_rotate(trans_ori, trans_add);
    return trans_after.translation;
}


common_msgs::CloudData NoizeCloudTransform::translation_noize(common_msgs::CloudData cloud, geometry_msgs::Vector3 translation)
{
    common_msgs::CloudData out_cloud;
    for (int i = 0; i < cloud.x.size(); i++) {
        out_cloud.x.push_back(cloud.x[i] + translation.x);
        out_cloud.y.push_back(cloud.y[i] + translation.y);
        out_cloud.z.push_back(cloud.z[i] + translation.z);
        out_cloud.instance.push_back(cloud.instance[i]);
    }
    out_cloud = UtilMsgData::substitute_cloudmsg_para(out_cloud, cloud);
    return out_cloud;
}

common_msgs::CloudData NoizeCloudTransform::translation_noize(common_msgs::CloudData cloud, float x, float y, float z)
{
    auto vector3 = UtilMsgData::vector3(x, y, z);
    return translation_noize(cloud, vector3);
}

common_msgs::CloudData NoizeCloudTransform::rotate_noize(common_msgs::CloudData cloud, geometry_msgs::Quaternion rotation)
{
    common_msgs::CloudData out_cloud;
    for (int i = 0; i < cloud.x.size(); i++) {
        tf2::Quaternion quat_ori(cloud.x[i], cloud.y[i], cloud.z[i], 0), quat_rotate;
        quat_rotate = TfFunction::geo_quat_to_tf2_quat(rotation);
        auto q_after = quat_rotate * quat_ori * quat_rotate.inverse();
        out_cloud.x.push_back(q_after[0]);
        out_cloud.y.push_back(q_after[1]);
        out_cloud.z.push_back(q_after[2]);
        out_cloud.instance.push_back(cloud.instance[i]);
    }
    out_cloud = UtilMsgData::substitute_cloudmsg_para(out_cloud, cloud);
    return out_cloud;
}

common_msgs::CloudData NoizeCloudTransform::rotate_noize(common_msgs::CloudData cloud, tf2::Quaternion quat)
{
    geometry_msgs::Quaternion rotation = TfFunction::tf2_quat_to_geo_quat(quat);
    return rotate_noize(cloud, rotation);
}

geometry_msgs::Vector3 NoizeCloudTransform::get_centroid(common_msgs::CloudData cloud)
{
    geometry_msgs::Vector3 centroid;
    int size = cloud.x.size();
    float xsum = 0, ysum = 0, zsum = 0;
    for (int i = 0; i < size; i++) {
        xsum += cloud.x[i];
        ysum += cloud.y[i];
        zsum += cloud.z[i];
    }
    centroid.x = xsum / size;
    centroid.y = ysum / size;
    centroid.z = zsum / size;
    return centroid;
}

common_msgs::CloudData NoizeCloudTransform::get_far_away_cloud(common_msgs::CloudData cloud, geometry_msgs::Vector3 centroid , int num)
{
    common_msgs::CloudData sort_cloud;
    sort_cloud = cloud;
    std::vector<int> sorted_index_list;
    for (int i = 0; i < cloud.x.size(); i++) {
        sorted_index_list.push_back(i);
    }
    int start_index = 0;
    std::vector<double> vec_ori = {centroid.x, centroid.y, centroid.z};
    for (int i = 0; i < num; i++) {
        double distance_max = 0;
        int max_index = start_index;
        for (int j = start_index; j < sort_cloud.x.size(); j++) {
            std::vector<double> vec1 = {sort_cloud.x[j], sort_cloud.y[j], sort_cloud.z[j]};
            auto dis = Util::distance(vec1, vec_ori);
            if (dis > distance_max) {
                distance_max = dis;
                max_index = j;
            }
        }
        std::swap(sorted_index_list[start_index], sorted_index_list[max_index]);
        std::swap(sort_cloud.x[start_index], sort_cloud.x[max_index]);
        std::swap(sort_cloud.y[start_index], sort_cloud.y[max_index]);
        std::swap(sort_cloud.z[start_index], sort_cloud.z[max_index]);
        std::swap(sort_cloud.instance[start_index], sort_cloud.instance[max_index]);
        start_index++;
    }
    common_msgs::CloudData final_cloud;
    for (int i = 0; i < num; i++) {
        int get_index = i;
        final_cloud.x.push_back(cloud.x[sorted_index_list[get_index]]);
        final_cloud.y.push_back(cloud.y[sorted_index_list[get_index]]);
        final_cloud.z.push_back(cloud.z[sorted_index_list[get_index]]);
        final_cloud.instance.push_back(cloud.instance[get_index]);
    }
    final_cloud = UtilMsgData::substitute_cloudmsg_para(final_cloud, cloud);
    return final_cloud;
}

common_msgs::CloudData NoizeCloudTransform::get_instance_boarder_cloud(common_msgs::CloudData cloud, double nearest_distance)
{
    common_msgs::CloudData out_cloud;
    for (int i = 0; i < cloud.x.size(); i++) {
        auto nearest_cloud = SpaceHandlingLibrary::search_nearest_point_on_unit(cloud, cloud, i, nearest_distance);
        for (int j = 0; j < nearest_cloud.x.size(); j++) {
            if (cloud.instance[i] != nearest_cloud.instance[j]) {
                out_cloud = UtilMsgData::pushback_cloud_point(cloud, i, out_cloud);
            }
        }
    }
    return out_cloud;
}