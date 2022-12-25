#include <noize_package/noize_cloud_transform.hpp>

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