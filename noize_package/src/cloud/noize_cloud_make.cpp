#include <noize_package/cloud/noize_cloud_make.hpp>

common_msgs::CloudData NoizeCloudMake::sphere_cloud(float r, int point_num)
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

common_msgs::CloudData NoizeCloudMake::sphere_empty_cloud(float r, int point_num)
{
    common_msgs::CloudData cloud;
    for (int i = 0; i < point_num; i++) {
        float x = util_.random_float(-r, r);
        float y, z;
        if (util_.probability() < 0.5) {
            y = std::sqrt(r*r - x*x);
        }
        else {
            y = -std::sqrt(r*r - x*x);
        }
        if (util_.probability() < 0.5) {
            z = std::sqrt(r*r - x*x - y*y);
        }
        else {
            z = -std::sqrt(r*r - x*x - y*y);
        }
        cloud.x.push_back(x);
        cloud.y.push_back(y);
        cloud.z.push_back(z);
        cloud.instance.push_back(0);
    }
    return cloud;
}

common_msgs::CloudData NoizeCloudMake::cylinder_cloud(float r, float z_max, int point_num)
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

common_msgs::CloudData NoizeCloudMake::rectangle_cloud(float width, float height, float dense, int point_num)
{
    common_msgs::CloudData cloud;
    for (int i = 0; i < point_num; i++) {
        float x = util_.random_float(-width, width);
        float y = util_.random_float(0, 2 * height);
        float z = util_.random_float(-dense, dense);
        cloud.x.push_back(x);
        cloud.y.push_back(y);
        cloud.z.push_back(z);
        cloud.instance.push_back(0);
    }
    return cloud;
}

common_msgs::CloudData NoizeCloudMake::make_defect_cloud(common_msgs::CloudData cloud, common_msgs::CloudData subtract_cloud, double nearest_distance)
{
    auto defect_cloud = SpaceHandlingLibrary::search_nearest_point(cloud, subtract_cloud, -1, nearest_distance);
    defect_cloud = UtilMsgData::remove_ins_cloudmsg(defect_cloud, -1);
    return defect_cloud;
}

common_msgs::CloudData NoizeCloudMake::noize_cloud_random()
{
    return noize_cloud_random(0.04, 0.04, 0.04);
}

common_msgs::CloudData NoizeCloudMake::noize_cloud_random(float x, float y, float z)
{
    common_msgs::CloudData noize_cloud, sum_cloud, noize_add;
    auto point_min = 2, point_max = 300;
    float min = x;
    if (min > y) {
        min = y;
    }
    if (min > z) {
        min = z;
    }
    float unit_min = 0.0003, unit_max = min / 1.5;
    for (int i = 0; i < util_.random_int(5, 40); i++) {
        auto probality = util_.probability();
        if (probality < 0.3) {
            noize_cloud = sphere_cloud(util_.random_float(unit_min, unit_max), util_.random_int(point_min, point_max));
        }
        else if (probality < 0.6){
            noize_cloud = cylinder_cloud(util_.random_float(unit_min, unit_max), util_.random_float(unit_min, unit_max), util_.random_int(point_min, point_max));
        }
        else {
            noize_cloud = rectangle_cloud(util_.random_float(unit_min, unit_max), util_.random_float(unit_min, unit_max), util_.random_float(unit_min, unit_max), util_.random_int(point_min, point_max));
        }
        geometry_msgs::Vector3 translation;
        translation.x = util_.random_float(-x, x);
        translation.y = util_.random_float(-y, y);
        translation.z = util_.random_float(-z, z);
        noize_add = NoizeCloudTransform::translation_noize(noize_cloud, translation);
        sum_cloud = UtilMsgData::concat_cloudmsg(sum_cloud, noize_add);
    }
    return sum_cloud;
}

common_msgs::CloudData NoizeCloudMake::circle_cloud(float radious, float angle_max, float tube_radious, int point_num)
{
    common_msgs::CloudData cloud;
    for (int i = 0; i < point_num; i++) {
        float x, y, z;
        auto angle = util_.random_float(0, angle_max);
        auto tube_angle = util_.random_float(0, 2 * M_PI);
        x = (radious + tube_radious * std::cos(tube_angle)) * std::cos(angle);
        y = (radious + tube_radious * std::cos(tube_angle)) * std::sin(angle);
        z = tube_radious * std::sin(tube_angle);
        cloud.x.push_back(x);
        cloud.y.push_back(y);
        cloud.z.push_back(z);
        cloud.instance.push_back(0);
    }
    return cloud;
}

common_msgs::CloudData NoizeCloudMake::linear_cloud(float length, float tube_radious, int point_num)
{
    common_msgs::CloudData cloud;
    for (int i = 0; i < point_num; i++) {
        float x, y, z;
        x = util_.random_float(-length / 2, length / 2);
        auto tube_angle = util_.random_float(0, 2 * M_PI);
        y = tube_radious * std::cos(tube_angle);
        z = tube_radious * std::sin(tube_angle);
        cloud.x.push_back(x);
        cloud.y.push_back(y);
        cloud.z.push_back(z);
        cloud.instance.push_back(0);
    }
    return cloud;
}

common_msgs::CloudData NoizeCloudMake::noize_tube_small_z_range(float z_range_min, float z_range_max)
{
    common_msgs::CloudData sum_cloud;
    auto base_range = 0.05;
    for (int i = 0; i < Util::random_int_static(1, 30); i++) {
        common_msgs::CloudData noize_part;
        if (util_.probability() < 0.5) {
            noize_part =  circle_cloud(util_.random_float(0.001, 0.025), util_.random_float(0, M_PI / 2), util_.random_float(0.0001, 0.001), Util::random_int_static(1, 500));
        }
        else {
            noize_part =  linear_cloud(util_.random_float(0.001, 0.025), util_.random_float(0.0001, 0.001), Util::random_int_static(1, 500));
        }
        auto quaternion = TfFunction::rotate_xyz_make(util_.random_float(-M_PI, M_PI), util_.random_float(-M_PI, M_PI), util_.random_float(-M_PI, M_PI));
        noize_part = NoizeCloudTransform::rotate_noize(noize_part, quaternion);
        noize_part = NoizeCloudTransform::translation_noize(noize_part, util_.random_float(-base_range, base_range), util_.random_float(-base_range, base_range), util_.random_float(z_range_min, z_range_max));
        noize_part = CloudProcess::cropbox_segmenter_origin_world(noize_part, base_range * 1.2, base_range * 1.2, 1);
        sum_cloud = UtilMsgData::concat_cloudmsg(sum_cloud, noize_part);
    }
    return sum_cloud;
}

common_msgs::CloudData NoizeCloudMake::noize_tube_small()
{
    common_msgs::CloudData noize_cloud;
    auto probable = util_.probability();
    if (probable < 0.33) {
        noize_cloud = noize_tube_small_z_range(0.005, 0.06);
    }
    else if (probable < 0.67) {
        noize_cloud = noize_tube_small_z_range(-0.06, -0.005);
    }
    else {
        noize_cloud = noize_tube_small_z_range(0.005, 0.06);
        auto another_noize = noize_tube_small_z_range(-0.06, -0.005);
        noize_cloud = UtilMsgData::concat_cloudmsg(noize_cloud, another_noize);
    }
    return noize_cloud;
}

common_msgs::CloudData NoizeCloudMake::noize_tube_big()
{
    common_msgs::CloudData sum_cloud;
    for (int i = 0; i < util_.random_int(0, 10); i++) {
        common_msgs::CloudData noize_part;
        noize_part =  circle_cloud(util_.random_float(0.01, 0.5), util_.random_float(0, M_PI * 3/ 2), util_.random_float(0.0001, 0.003), Util::random_int_static(100, 2000));
        auto quaternion = TfFunction::rotate_xyz_make(util_.random_float(-M_PI, M_PI), util_.random_float(-M_PI, M_PI), util_.random_float(-M_PI, M_PI));
        noize_part = NoizeCloudTransform::rotate_noize(noize_part, quaternion);
        noize_part = CloudProcess::cropbox_segmenter_origin_world(noize_part, 0.1, 0.1, 1);
        noize_part = NoizeCloudTransform::translation_noize(noize_part, util_.random_float(-0.1, 0.1), util_.random_float(-0.1, 0.1), util_.random_float(-1, 1));
        sum_cloud = UtilMsgData::concat_cloudmsg(sum_cloud, noize_part);
    }
    return sum_cloud;
}