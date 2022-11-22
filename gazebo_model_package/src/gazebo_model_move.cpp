#include <gazebo_model_package/gazebo_model_move.hpp>

GazeboMoveServer::GazeboMoveServer(ros::NodeHandle nh) : nh_(nh), pnh_("~")
                                                    
{
    gazebo_service_name_ = "/gazebo/set_model_state";
    gazebo_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(gazebo_service_name_);
    server_ = nh_.advertiseService(gazebo_service_name_, &GazeboMoveServer::service_callback, this);
}


void GazeboMoveServer::set_multi_gazebo_model(std::vector<common_msgs::ObjectInfo> multi_object_info)
{
    for (int i = 0; i < multi_object_info.size(); i++)
    {
        gazebo_msgs::SetModelState multi_gazebo;
        multi_gazebo.request.model_state = UtilMsgData::make_gazebo_model_state(multi_object_info[i]);
        for (int j = 0; j < 3; j++)
        {
            Util::client_request(gazebo_client_, multi_gazebo, gazebo_service_name_);
            ros::Duration(0.001).sleep();
        }
    }
}

void GazeboMoveServer::set_gazebo_model(common_msgs::ObjectInfo object_info)
{
    for (int j = 0; j < 3; j++)
    {
        gazebo_msgs::SetModelState gazebo_srv;
        gazebo_srv.request.model_state = UtilMsgData::make_gazebo_model_state(object_info);
        Util::client_request(gazebo_client_, gazebo_srv, gazebo_service_name_);
        ros::Duration(0.01).sleep();
    }
}

bool GazeboMoveServer::service_callback(common_srvs::GazeboSensorMoveServiceRequest &request, common_srvs::GazeboSensorMoveServiceResponse &response)
{
    set_gazebo_model(request.object_info);
    tf2::Quaternion quat = TfBasic::make_tf2_quaternion(request.object_info.position.rotation);
    quat = TfBasic::rotate_quaternion_by_axis(quat, RotationOption::y, M_PI/2) * quat;
    geometry_msgs::Transform trans = request.object_info.position;
    trans.rotation = TfBasic::make_geo_quaternion(quat);
    geometry_msgs::TransformStamped trans_stamp;
    trans_stamp = TfBasic::make_geo_trans_stamped(request.object_info.tf_name, world_frame_, trans);
    tf_basic_.static_broadcast(trans_stamp);
}

void GazeboMoveServer::set_parameter()
{
    pnh_.getParam("gazebo_move_server", param_list);
    gazebo_service_name_ = static_cast<std::string>(param_list["gazebo_service_name"]);
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
}

