#pragma once
#include <anno_srvs/MeshCloudService.h>
#include <anno_srvs/RecordService.h>
#include <common_srvs/SensorService.h>
#include <gazebo_model_package/decide_object_position.hpp>
#include <gazebo_model_package/gazebo_model_move.hpp>
#include <tf_package/tf_basic.hpp>
#include <util/util_base.hpp>
#include <labeling_package/instance_label_drawer.hpp>
#include <data_convert_pkg/make_2Dinfo_by_3D.hpp>
#include <data_convert_pkg/get_3D_by_2D.hpp>
#include <data_convert_pkg/func_data_convertion.hpp>
#include <util_anno/util_anno.hpp>

class SemanticSegmentation
{
public:
    SemanticSegmentation(ros::NodeHandle &);
    void main();
    void set_paramenter();
    void visualize_publish();
    XmlRpc::XmlRpcValue param_list;

private:
    ros::NodeHandle nh_, pnh_;
    ros::ServiceClient sensor_client_, mesh_client_;
    std::string sensor_service_name_, mesh_service_name_;
    std::string world_frame_, sensor_frame_;
    std::string pc_pub_topic_;
    TfBasic tf_basic_;
    sensor_msgs::PointCloud2 pc_visualize_data_;
    UtilSensor util_sensor_;
    ros::Publisher pc_pub_;
    InstanceLabelDrawer instance_drawer_;
    double nearest_radious_;
};