#include <gazebo_model_package/gazebo_model_move.hpp>

GazeboMoveServer::GazeboMoveServer(ros::NodeHandle nh) : nh_(nh), pnh_("~")
                                                    
{
    gazebo_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
}


void GazeboMoveServer::set_multi_gazebo_model(std::vector<common_msgs::ObjectInfo> multi_object_info)
{
    for (int i = 0; i < multi_object_info.size(); i++)
    {
        gazebo_msgs::ModelState model_state = UtilMsgData::make_gazebo_model_state(multi_object_info[i]);
        for (int j = 0; j < 5; j++)
        {
            gazebo_pub_.publish(model_state);
            ros::Duration(0.001).sleep();
        }
    }
}

void GazeboMoveServer::set_gazebo_model(common_msgs::ObjectInfo object_info)
{
    for (int j = 0; j < 5; j++)
    {
        gazebo_msgs::ModelState model_state = UtilMsgData::make_gazebo_model_state(object_info);
        gazebo_pub_.publish(model_state);
        ros::Duration(0.01).sleep();
    }
}


void GazeboMoveServer::set_parameter()
{
    pnh_.getParam("gazebo_move_server", param_list);
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
}

