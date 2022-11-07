#pragma once
#include <ros/ros.h>
#include <pcl_ros/impl/transforms.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include "mesh_sampler.hpp"
#include <tf/transform_datatypes.h>
#include <anno_srvs/MeshCloudService.h>
#include <common_msgs/CloudData.h>
#include <common_msgs/PoseData.h>
#include <util/util_base.hpp>
#include <util/util_sensor.hpp>
#include <tf_package/tf_basic.hpp>



struct MeshOutType
{
    std::vector<common_msgs::CloudData> mesh_data;
    std::vector<common_msgs::PoseData> pose_data;
};
class MeshCloudServer
{
public:
    MeshCloudServer(ros::NodeHandle &);
    void set_parameter();
    MeshOutType make_mesh(anno_srvs::MeshCloudServiceRequest);
    bool service_callback(anno_srvs::MeshCloudServiceRequest &, anno_srvs::MeshCloudServiceResponse &);
    void visualize_callback(const ros::TimerEvent &);
    void visualize_data(int);
    void initialize(anno_srvs::MeshCloudServiceRequest);
    common_msgs::PoseData stamped_to_pose(tf::StampedTransform);
    XmlRpc::XmlRpcValue param_list;
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceServer server_, visual_server_;
    std::vector<ros::Publisher> mesh_cluster_pub_;
    std::vector<pcl::PointCloud<PclXyz>> mesh_pcl_clusters_;
    ros::Timer timer_;
    std::string world_frame_, mesh_service_name_, sensor_frame_;
    int sample_points;
    double LEAF_SIZE_;
};
