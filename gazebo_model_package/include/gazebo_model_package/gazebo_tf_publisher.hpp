#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf_package/tf_function.hpp>
#include <common_srvs/TfBroadcastService.h>
#include <util/util.hpp>

class GazeboTfPublisher
{
public:
    GazeboTfPublisher(ros::NodeHandle&);
    void modelstatesCallback(const gazebo_msgs::ModelStates::ConstPtr&);

private:
    ros::NodeHandle nh_, pnh_;
    XmlRpc::XmlRpcValue param_list;
    std::string world_frame_;
    ros::Subscriber model_state_sub_;
    std::vector<std::string> model_names_;
    std::vector<geometry_msgs::Pose> model_poses_;
    std::string gazebo_tracked_frame_, rviz_following_frame_;
    TfFunction tf_func_;
    ros::ServiceClient tf_client_;
    std::string tf_br_service_name_;
    void set_parameter();
};
