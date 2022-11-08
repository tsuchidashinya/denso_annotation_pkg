#include <annotation_client_pkg/semantic_segmentation_client.hpp>

SemanticSegmentation::SemanticSegmentation(ros::NodeHandle &nh):
    nh_(nh),
    pnh_("~")
{
    set_paramenter();
    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
    mesh_client_ = nh_.serviceClient<anno_srvs::MeshCloudService>(mesh_service_name_);
}

void SemanticSegmentation::set_paramenter()
{
    pnh_.getParam("annotation_main", param_list);
    sensor_service_name_ = static_cast<std::string>(param_list["sensor_service_name"]);
}

void SemanticSegmentation::main()
{
    // common_srvs::SensorService sensor_srv;
    // sensor_srv.request.counter = 1;
    // UtilBase::client_request(sensor_client_, sensor_srv, sensor_service_name_);

    DecidePosition decide_gazebo_object;
    GazeboModelMove gazebo_model_move(nh_);
    std::vector<anno_msgs::ObjectInfo> multi_object;
    for (int i = 0; i < 10; i++) {
        anno_msgs::ObjectInfo object;
        object = decide_gazebo_object.make_object_info(i, "HV8");
        multi_object.push_back(object);
    }
    multi_object = decide_gazebo_object.get_randam_place_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    
    anno_srvs::MeshCloudService mesh_srv;
    mesh_srv.request.multi_object_info = multi_object;
    UtilBase::client_request(mesh)

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_segmentation");
    ros::NodeHandle nh;
    SemanticSegmentation semseg(nh);
    for (int i = 0; i < 20; i++) {
        semseg.main();
        ros::Duration(20).sleep();
    }
    
    return 0;
}