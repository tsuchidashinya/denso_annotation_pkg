#include <gazebo_model_package/gazebo_model_server.hpp>

GazeboModelServer::GazeboModelServer(ros::NodeHandle nh) : nh_(nh),
                                                           pnh_("~")
{
    pnh_.getParam("gazebo_model_server", param_list);
}

void GazeboModelServer::set_multi_gazebo_model(std::vector<gazebo_msgs::ModelState> gazebo_models)
{
    for (int i = 0; i < gazebo_models.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            UtilBase::client_request(gazebo_client_, gazebo_models[i], "/gazebo/set_model_state");
            ros::Duration(0.001).sleep();
        }
    }
}

void GazeboModelServer::set_gazebo_model(gazebo_msgs::ModelState gazebo_model)
{
    for (int j = 0; j < 3; j++)
    {
        UtilBase::client_request(gazebo_client_, gazebo_model, "/gazebo/set_model_state");
        ros::Duration(0.001).sleep();
    }
}

bool GazeboModelServer::service_callback()