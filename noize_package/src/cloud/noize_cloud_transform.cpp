#include <noize_package/cloud/noize_cloud_transform.hpp>

common_msgs::CloudData NoizeCloudTransform::translation_noize(common_msgs::CloudData cloud, geometry_msgs::Vector3 translation)
{
    common_msgs::CloudData out_cloud;
    for (int i = 0; i < cloud.x.size(); i++) {
        out_cloud.x.push_back(cloud.x[i] + translation.x);
        out_cloud.y.push_back(cloud.y[i] + translation.y);
        out_cloud.z.push_back(cloud.z[i] + translation.z);
        out_cloud.instance.push_back(cloud.instance[i]);
    }
    out_cloud.object_name = cloud.object_name;
    out_cloud.tf_name = cloud.tf_name;
    out_cloud.frame_id = cloud.frame_id;
    return out_cloud;
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
    out_cloud.object_name = cloud.object_name;
    out_cloud.tf_name = cloud.tf_name;
    out_cloud.frame_id = cloud.frame_id;
    return out_cloud;
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
    std::vector<int> sorted_index_list;
    for (int i = 0; i < cloud.x.size(); i++) {
        sorted_index_list.push_back(i);
    }
    int start_index = 0;
    std::vector<double> vec_ori = {centroid.x, centroid.y, centroid.z};
    for (int i = 0; i < num; i++) {
        double distance_max;
        int max_index = start_index;
        for (int j = start_index; j < cloud.x.size(); j++) {
            if (j == start_index) {
                std::vector<double> vec = {cloud.x[start_index], cloud.y[start_index], cloud.z[start_index]};
                distance_max = Util::distance(vec, vec_ori);
            }
            std::vector<double> vec1 = {cloud.x[i], cloud.y[i], cloud.z[i]};
            auto dis = Util::distance(vec1, vec_ori);
            if (dis > distance_max) {
                distance_max = dis;
                max_index = i;
            }
        }
        std::swap(sorted_index_list[start_index], sorted_index_list[max_index]);
        start_index++;
    }
    common_msgs::CloudData final_cloud;
    for (int i = 0; i < num; i++) {
        final_cloud.x.push_back(cloud.x[sorted_index_list[i]]);
        final_cloud.y.push_back(cloud.y[sorted_index_list[i]]);
        final_cloud.z.push_back(cloud.z[sorted_index_list[i]]);
        final_cloud.instance.push_back(cloud.instance[i]);
    }
    final_cloud.frame_id = cloud.frame_id;
    final_cloud.object_name = cloud.object_name;
    final_cloud.tf_name = cloud.tf_name;
    return final_cloud;
}