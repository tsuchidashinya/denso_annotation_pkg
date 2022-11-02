#pragma once
#include <anno_srvs/GazeboService.h>
#include <anno_srvs/MeshCloudService.h>
#include <anno_srvs/RecordService.h>
#include <common_srvs/SensorService.h>

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
    ros::ServiceClient sensor_client_;
};