#include <noize_package/noize_cloud_make.hpp>

common_msgs::CloudData NoizeCloudMake::sphere_unit(float r, int point_num)
{
    common_msgs::CloudData cloud;
    for (int i = 0; i < point_num; i++) {
        float x = util_.random_float(-r, r);
        float y = util_.random_float(-std::sqrt(r*r - x*x), std::sqrt(r*r - x*x));
        float z = util_.random_float(-std::sqrt(r*r - x*x - y*y), std::sqrt(r*r - x*x - y*y));
        cloud.x.push_back(x);
        cloud.y.push_back(y);
        cloud.z.push_back(z);
        cloud.instance.push_back(0);
    }
    return cloud;
}

common_msgs::CloudData NoizeCloudMake::cylinder_unit(float r, float z_max, int point_num)
{
    common_msgs::CloudData cloud;
    for (int i = 0; i < point_num; i++) {
        float x = util_.random_float(-r, r);
        float y = util_.random_float(-std::sqrt(r*r - x*x), std::sqrt(r*r - x*x));
        float z = util_.random_float(-z_max, z_max);
        cloud.x.push_back(x);
        cloud.y.push_back(y);
        cloud.z.push_back(z);
        cloud.instance.push_back(0);
    }
    return cloud;
}