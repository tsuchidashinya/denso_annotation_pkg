#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/transform_broadcaster.h>



class GazeboTfPublisher
{
public:
    GazeboTfPublisher(ros::NodeHandle&);
    void modelstatesCallback(const gazebo_msgs::ModelStates::ConstPtr&);
    void broadcastModelTF();
private:
    ros::NodeHandle nh_;
    std::string src_frame_, urdf_model_name_;
    ros::Subscriber model_state_sub_;
    std::vector<std::string> model_names_;
    std::vector<geometry_msgs::Pose> model_poses_;

};
