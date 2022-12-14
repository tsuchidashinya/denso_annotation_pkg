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
    return out_cloud;
}