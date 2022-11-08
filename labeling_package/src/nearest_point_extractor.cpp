#include <labeling_package/nearest_point_extractor.hpp>

common_msgs::CloudData NearestPointExtractor::draw_initial_instance(common_msgs::CloudData cloud, int itial_instance)
{
    UtilSensor::cloud_size_ok(cloud);
    for (int i = 0; i < cloud.x.size(); i++) {
        cloud.instance[i] = itial_instance;
    }
    return cloud;
}

common_msgs::CloudData NearestPointExtractor::extract_nearest_point(common_msgs::CloudData sensor_cloud, common_msgs::CloudData mesh_cloud, int instance, double radious = 0.003)
{
    pcl::PointCloud<PclXyz> sensor_pcl, mesh_pcl;
    sensor_pcl = UtilSensor::cloudmsg_to_pcl(sensor_cloud);
    mesh_pcl = UtilSensor::cloudmsg_to_pcl(mesh_cloud);
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