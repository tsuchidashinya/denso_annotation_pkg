#include <gazebo_model_package/gazebo_model_move.hpp>

GazeboModelMove::GazeboModelMove(ros::NodeHandle nh) : nh_(nh)
                                                    
{
    gazebo_service_name_ = "/gazebo/set_model_state";
    gazebo_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(gazebo_service_name_);
}


void GazeboModelMove::set_multi_gazebo_model(std::vector<anno_msgs::ObjectInfo> multi_object_info)
{
    for (int i = 0; i < multi_object_info.size(); i++)
    {
        gazebo_msgs::SetModelState multi_gazebo;
        multi_gazebo.request.model_state = make_gazebo_model_state(multi_object_info[i]);
        for (int j = 0; j < 3; j++)
        {
            UtilBase::client_request(gazebo_client_, multi_gazebo, gazebo_service_name_);
            ros::Duration(0.001).sleep();
        }
    }
}

void GazeboModelMove::set_gazebo_model(anno_msgs::ObjectInfo object_info)
{
    for (int j = 0; j < 3; j++)
    {
        gazebo_msgs::SetModelState gazebo_srv;
        gazebo_srv.request.model_state = make_gazebo_model_state(object_info);
        UtilBase::client_request(gazebo_client_, gazebo_srv, gazebo_service_name_);
        ros::Duration(0.001).sleep();
    }
}

/**
 * @brief gazebo_modelメッセージを生成する関数です。
 *
 * @param object_name Gazeboオブジェクトの名前
 * @param trans geometry_msgs::Transform型の姿勢データ
 * @return gazebo_msgs::ModelState
 */
gazebo_msgs::ModelState GazeboModelMove::make_gazebo_model_state(std::string object_name, geometry_msgs::Transform trans)
{
    gazebo_msgs::ModelState model;
    model.model_name = object_name;
    model.pose.position.x = trans.translation.x;
    model.pose.position.y = trans.translation.y;
    model.pose.position.z = trans.translation.z;
    tf2::convert(trans.rotation, model.pose.orientation);
    return model;
}

gazebo_msgs::ModelState GazeboModelMove::make_gazebo_model_state(anno_msgs::ObjectInfo object_info)
{
    gazebo_msgs::ModelState model;
    model.model_name = object_info.tf_name;
    model.pose.position.x = object_info.position.translation.x;
    model.pose.position.y = object_info.position.translation.y;
    model.pose.position.z = object_info.position.translation.z;
    tf2::convert(object_info.position.rotation, model.pose.orientation);
    return model;
}

