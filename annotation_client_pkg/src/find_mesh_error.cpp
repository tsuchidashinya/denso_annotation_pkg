#include <annotation_client_pkg/annotation_client.hpp>


AnnotationClient::AnnotationClient(ros::NodeHandle &nh):
    nh_(nh),
    pnh_("~")
{
    set_paramenter();
    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
    mesh_client_ = nh_.serviceClient<common_srvs::MeshCloudService>(mesh_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
    record_client_ = nh_.serviceClient<common_srvs::RecordSegmentation>(record_service_name_);
    vis_delete_client_ = nh_.serviceClient<common_srvs::VisualizeCloudDelete>(vis_delete_service_name_);
}

void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    nearest_radious_ = param_list["nearest_radious"];
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    vis_delete_service_name_ = static_cast<std::string>(param_list["visualize_delete_service_name"]);
    sensor_service_name_ = static_cast<std::string>(param_list["sensor_service_name"]);
    mesh_service_name_ = static_cast<std::string>(param_list["mesh_service_name"]);
    record_service_name_ = static_cast<std::string>(param_list["record_service_name"]);
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    gazebo_sensor_service_name_ = static_cast<std::string>(param_list["gazebo_sensor_service_name"]);
    occlusion_object_radious_ = param_list["occlusion_object_radious"];
    object_list_.push_back(static_cast<std::string>(param_list["main_object_name"]));
    quantity_of_object_list_.push_back(static_cast<int>(param_list["quantity_of_main_object"]));
    instance_of_object_list_.push_back(static_cast<int>(param_list["instance_of_main_object"]));
    pnh_.getParam("annotation_main/other_object_list", param_list);
    for (int i = 0; i < param_list.size(); i++) {
        object_list_.push_back(static_cast<std::string>(param_list[i]["object_name"]));
        quantity_of_object_list_.push_back(static_cast<int>(param_list[i]["quantity_of_the_object"]));
        instance_of_object_list_.push_back(static_cast<int>(param_list[i]["instance_of_the_object"]));
    }
}

void AnnotationClient::main()
{
    for (int i = 0; i < 1000; i++) {
        DecidePosition decide_gazebo_object;
        GazeboMoveServer gazebo_model_move(nh_);
        std::vector<common_msgs::ObjectInfo> multi_object, multi_object_all;
        for (int i = 0; i < 1; i++) {
            for (int i = 0; i < object_list_.size(); i++) {
                for (int j = 0; j < quantity_of_object_list_[i]; j++) {
                    common_msgs::ObjectInfo object;
                    object = decide_gazebo_object.make_object_info(j, object_list_[i]);
                    multi_object_all.push_back(object);
                }
            }
        }
        Util::message_show("client_counter: ", i);
        
        
        multi_object_all = decide_gazebo_object.get_remove_position(multi_object_all);
        common_srvs::MeshCloudService mesh_srv;
        mesh_srv.request.multi_object_info = multi_object_all;
        Util::client_request(mesh_client_, mesh_srv, mesh_service_name_);
    }
}