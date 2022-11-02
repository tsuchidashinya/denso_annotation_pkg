#pragma once
#include <ros/ros.h>
#include <pcl_ros/impl/transforms.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <mesh_cloud_package/mesh_sampler.hpp>
#include <tf/transform_datatypes.h>
#include <anno_srvs/MeshCloudService.h>
#include <common_msgs/CloudData.h>
#include <common_msgs/PoseData.h>

using namespace pcl;
using namespace std;

struct mesh_out_type
{
    vector<common_msgs::CloudData> mesh_data;
    vector<common_msgs::PoseData> pose_data;
};
class MeshCloudServer 
{
public:
    MeshCloudServer(ros::NodeHandle &);
    void set_parameter();
    mesh_out_type make_mesh(anno_srvs::MeshCloudServiceRequest);
    bool service_callback(anno_srvs::MeshCloudServiceRequest &, anno_srvs::MeshCloudServiceResponse &);
    void visualize_data(int);
    void initialize();
    common_msgs::PoseData stamped_to_pose(tf::StampedTransform);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceServer server_, visual_server_;
    geometry_msgs::TransformStamped tf_save_;
    sensor_msgs::PointCloud2 mesh_cloud_ros_;
    vector<ros::Publisher> mesh_cluster_pub_;
    PointCloud<PointXYZ> mesh_point_pcl_;
    vector<PointCloud<PointXYZ>> mesh_pcl_clusters_;
    vector<sensor_msgs::PointCloud2> mesh_ros_;
    vector<PolygonMesh> meshes_cluster_;
    vector<double> GT_translation_, GT_rotation_;
    tf::StampedTransform transform_;
    string object_name_, mesh_path_, mesh_topic_name_, world_frame_, mesh_service_name_, sensor_frame_;
    int sample_points, background_instance_;
    double LEAF_SIZE_;
};
