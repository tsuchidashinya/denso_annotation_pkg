#include <gazebo_model_package/gazebo_model_move.hpp>

GazeboModelMove::GazeboModelMove(ros::NodeHandle nh) : nh_(nh),
                                                           pnh_("~")
{
    pnh_.getParam("gazebo_model_move", param_list);
    set_parameter();
    gazebo_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}

void GazeboModelMove::set_parameter()
{
    service_name_ = static_cast<std::string>(param_list["service_name"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    object_name_ = static_cast<std::string>(param_list["object_name"]);
    object_num_ = static_cast<int>(param_list["object_num"]);
}

void GazeboModelMove::set_multi_gazebo_model(std::vector<gazebo_msgs::ModelState> gazebo_models)
{
    for (int i = 0; i < gazebo_models.size(); i++)
    {
        gazebo_msgs::SetModelState multi_gazebo;
        multi_gazebo.request.model_state = gazebo_models[i];
        for (int j = 0; j < 3; j++)
        {
            UtilBase::client_request(gazebo_client_, multi_gazebo, "/gazebo/set_model_state");
            ros::Duration(0.001).sleep();
        }
    }
}

void GazeboModelMove::set_gazebo_model(gazebo_msgs::ModelState gazebo_model)
{
    for (int j = 0; j < 3; j++)
    {
        gazebo_msgs::SetModelState gazebo_srv;
        gazebo_srv.request.model_state = gazebo_model;
        UtilBase::client_request(gazebo_client_, gazebo_srv, "/gazebo/set_model_state");
        ros::Duration(0.001).sleep();
    }
}

