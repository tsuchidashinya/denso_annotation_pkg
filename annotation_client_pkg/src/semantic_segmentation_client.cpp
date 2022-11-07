#include <annotation_client_pkg/semantic_segmentation_client.hpp>

SemanticSegmentation::SemanticSegmentation(ros::NodeHandle &nh):
    nh_(nh),
    pnh_("~")
{
    set_paramenter();
    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
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

    anno_srvs::GazeboService gazebo_srv;
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_segmentation");
    ros::NodeHandle nh;
    SemanticSegmentation semseg(nh);
    for (int i = 0; i < 20; i++) {
        semseg.main();
        ros::Duration(0.4).sleep();
    }
    
    return 0;
}