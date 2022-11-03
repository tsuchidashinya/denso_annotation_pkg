#include <gazebo_model_package/gazebo_model_server.hpp>

GazeboModelServer::GazeboModelServer(ros::NodeHandle nh) : nh_(nh),
                                                           pnh_("~")
{
    pnh_.getParam("gazebo_model_server", param_list);
    set_parameter();
    gazebo_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    server_ = nh_.advertiseService(service_name_, &GazeboModelServer::service_callback, this);
}

void GazeboModelServer::set_parameter()
{
    service_name_ = static_cast<std::string>(param_list["service_name"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    object_name_ = static_cast<std::string>(param_list["object_name"]);
    object_num_ = static_cast<int>(param_list["object_num"]);
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

bool GazeboModelServer::service_callback(anno_srvs::GazeboService::Request &request, anno_srvs::GazeboService::Response &response)
{
    DecidePosition decide_position;
    GazeboModelType phoxy, box;

    phoxy = decide_position.get_phoxi_position(M_PI / 36, M_PI / 26, 1);
    set_gazebo_model(phoxy.gazebo_model);

    box = decide_position.get_box_position(0);
    set_gazebo_model(box.gazebo_model);

    GazeboModelMultiType objects;
    for (int i = 0; i < object_num_; i++)
    {
        objects.object_infoes.push_back(decide_position.register_object(i, object_name_));
    }

    objects = decide_position.get_remove_position(objects.object_infoes);
    set_multi_gazebo_model(objects.gazebo_models);

    objects = decide_position.get_ramdam_place_position(objects.object_infoes);
    set_multi_gazebo_model(objects.gazebo_models);

    response.out_data = objects.object_infoes;
    return true;
}