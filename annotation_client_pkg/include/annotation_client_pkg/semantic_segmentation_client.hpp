#pragma once
#include <anno_srvs/MeshCloudService.h>
// #include <anno_srvs/RecordService.h>
#include <common_srvs/SensorService.h>
#include <common_srvs/VisualizeCloud.h>
#include <anno_srvs/RecordSegmentation.h>
#include <gazebo_model_package/decide_object_position.hpp>
#include <gazebo_model_package/gazebo_model_move.hpp>
#include <tf_package/tf_basic.hpp>
#include <util/util_base.hpp>
#include <labeling_package/instance_label_drawer.hpp>
#include <data_transform_pkg/make_2Dinfo_by_3D.hpp>
#include <data_transform_pkg/get_3D_by_2D.hpp>
#include <data_transform_pkg/func_data_convertion.hpp>
#include <annotation_common/util_anno.hpp>

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
    ros::ServiceClient sensor_client_, mesh_client_, visualize_client_, record_client_;
    std::string sensor_service_name_, mesh_service_name_, visualize_service_name_, record_service_name_;
    std::string world_frame_, sensor_frame_;
    TfBasic tf_basic_;
    UtilSensor util_sensor_;
    InstanceLabelDrawer instance_drawer_;
    double nearest_radious_;
    int the_number_of_dataset_;
};