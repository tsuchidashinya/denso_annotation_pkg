#pragma once
#include <anno_srvs/MeshCloudService.h>
#include <anno_srvs/RecordService.h>
#include <common_srvs/SensorService.h>
#include <gazebo_model_package/decide_object_position.hpp>
#include <gazebo_model_package/gazebo_model_move.hpp>
#include <tf_package/tf_basic.hpp>
#include <util/util_base.hpp>

class SemanticSegmentation
{
public:
    SemanticSegmentation(ros::NodeHandle &);
    void main();
    void set_paramenter();
    XmlRpc::XmlRpcValue param_list;

private:
    ros::NodeHandle nh_, pnh_;
    ros::ServiceClient sensor_client_, mesh_client_;
    std::string sensor_service_name_, mesh_service_name_;
};