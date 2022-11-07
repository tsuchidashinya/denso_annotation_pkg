#include <gazebo_model_package/gazebo_tf_publisher.hpp>
#include <algorithm>



GazeboTfPublisher::GazeboTfPublisher(ros::NodeHandle &nh) : nh_(nh)
{
    model_state_sub_ = nh_.subscribe("/gazebo/model_states", 1, &GazeboTfPublisher::modelstatesCallback, this);
}

void GazeboTfPublisher::modelstatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    model_names_ = msg->name;
    model_poses_ = msg->pose;
    broadcastGazeboTfPublisher();
}

void GazeboTfPublisher::broadcastGazeboTfPublisher()
{
    while (brs_.size() < model_names_.size()) {
        tf2_ros::TransformBroadcaster br;
        brs_.push_back(br);
    }
    ros::Rate loop(100);
    for (int i = 0; i < model_names_.size(); i++) {
        geometry_msgs::TransformStamped tf_stamp;
        
        tf_stamp.header.frame_id = "world";
        tf_stamp.child_frame_id = model_names_[i];
        tf_stamp.transform.translation.x = model_poses_[i].position.x;
        tf_stamp.transform.translation.y = model_poses_[i].position.y;
        tf_stamp.transform.translation.z = model_poses_[i].position.z;
        tf_stamp.transform.rotation.x = model_poses_[i].orientation.x;
        tf_stamp.transform.rotation.y = model_poses_[i].orientation.y;
        tf_stamp.transform.rotation.z = model_poses_[i].orientation.z;
        tf_stamp.transform.rotation.w = model_poses_[i].orientation.w;
        tf_stamp.header.stamp = ros::Time::now();
        brs_[i].sendTransform(tf_stamp);
        loop.sleep();
    }
}
