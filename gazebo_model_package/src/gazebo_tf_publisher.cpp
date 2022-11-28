#include <gazebo_model_package/gazebo_tf_publisher.hpp>
#include <algorithm>



GazeboTfPublisher::GazeboTfPublisher(ros::NodeHandle &nh) : nh_(nh), pnh_("~")
{
    set_parameter();
    model_state_sub_ = nh_.subscribe("/gazebo/model_states", 1, &GazeboTfPublisher::modelstatesCallback, this);
}

void GazeboTfPublisher::set_parameter()
{
   pnh_.getParam("gazebo_tf_publisher", param_list);
   world_frame_ = static_cast<std::string>(param_list["world_frame"]);
   gazebo_tracked_frame_ = static_cast<std::string>(param_list["gazebo_tracked_frame"]);
   rviz_following_frame_ = static_cast<std::string>(param_list["rviz_following_frame"]);
}

void GazeboTfPublisher::modelstatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    model_names_ = msg->name;
    model_poses_ = msg->pose;
    for (int i = 0; i < model_names_.size(); i++) {
        
        geometry_msgs::TransformStamped tf_stamp;
        geometry_msgs::Transform trans;
        tf2::Quaternion quat = TfFunction::make_tf2_quaternion(model_poses_[i].orientation);
        trans = TfFunction::make_geo_transform(model_poses_[i].position.x, model_poses_[i].position.y, model_poses_[i].position.z, quat);
        tf_stamp = TfFunction::make_geo_trans_stamped(model_names_[i], world_frame_, trans);
        tf_func_.static_broadcast(tf_stamp);
        if (model_names_[i] == gazebo_tracked_frame_) {
            tf_stamp.child_frame_id = rviz_following_frame_;
            tf_stamp.transform.rotation = TfFunction::make_geo_quaternion(TfFunction::rotate_xyz_make(0, M_PI/2, 0, TfFunction::make_tf2_quaternion(tf_stamp.transform.rotation)));
            tf_func_.static_broadcast(tf_stamp);
        }
    }

}
