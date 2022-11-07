#include <labeling_package/nearest_point_extractor.hpp>

common_msgs::CloudData NearestPointExtractor::draw_initial_instance(common_msgs::CloudData cloud, int itial_instance)
{
    UtilSensor::cloud_size_ok(cloud);
    for (int i = 0; i < cloud.x.size(); i++) {
        cloud.instance[i] = itial_instance;
    }
    return cloud;
}

common_msgs::CloudData NearestPointExtractor::extract_nearest_point(common_msgs::CloudData sensor_cloud, common_msgs::CloudData mesh_cloud, int instance)
{

}