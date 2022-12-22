#include <annotation_client_pkg/annotation_client.hpp>


AnnotationClient::AnnotationClient(ros::NodeHandle &nh):
    nh_(nh),
    pnh_("~")
{
    set_paramenter();
    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
    mesh_client_ = nh_.serviceClient<common_srvs::MeshCloudService>(mesh_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
    hdf5_record_client_ = nh_.serviceClient<common_srvs::Hdf5RecordRealSensorData>(hdf5_record_service_name_);
    vis_img_client_ = nh_.serviceClient<common_srvs::VisualizeImage>(vis_img_service_name_);
}

void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    sensor_service_name_ = static_cast<std::string>(param_list["sensor_service_name"]);
    hdf5_record_service_name_ = static_cast<std::string>(param_list["hdf5_record_service_name"]);
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    vis_img_service_name_ = static_cast<std::string>(param_list["visualize_image_service_name"]);
    hdf5_record_file_path__ = static_cast<std::string>(param_list["hdf5_record_file_path"]);
}

void AnnotationClient::main()
{
    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 1;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    common_msgs::CloudData sensor_cloud = sensor_srv.response.cloud_data;
    cv::Mat img = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    std::vector<float> cinfo_list = UtilMsgData::caminfo_to_floatlist(sensor_srv.response.camera_info);
    sensor_cloud = InstanceLabelDrawer::draw_instance_all(sensor_cloud, 0);
    common_srvs::Hdf5RecordRealSensorData record_srv;
    record_srv.request.record_file_path = hdf5_record_file_path__;
    record_srv.request.camera_info = cinfo_list;
    record_srv.request.image = sensor_srv.response.image;
    record_srv.request.cloud_data = sensor_cloud;
    record_srv.request.the_number_of_dataset = the_number_of_dataset_;
    Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
    common_srvs::VisualizeCloud visualize_srv;
    visualize_srv.request.cloud_data_list.push_back(sensor_cloud);
    visualize_srv.request.topic_name_list.push_back("acc_cloud");
    Util::client_request(visualize_client_, visualize_srv, visualize_service_name_);
    common_srvs::VisualizeImage vis_img_srv;
    vis_img_srv.request.image_list.push_back(sensor_srv.response.image);
    vis_img_srv.request.topic_name_list.push_back("real_image");
    Util::client_request(vis_img_client_, vis_img_srv, vis_img_service_name_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_segmentation");
    ros::NodeHandle nh;
    AnnotationClient annotation_main(nh);
    for (int i = 0; i < annotation_main.the_number_of_dataset_; i++) {
        annotation_main.main();
        // ros::Duration(0.1).sleep();
        Util::message_show("Progress rate", std::to_string(i + 1) + "/" + std::to_string(annotation_main.the_number_of_dataset_));
        std::cout << "key input wait" << std::endl;
        std::string input;
        std::cin >> input;
        std::cout << input << std::endl;
    }
    return 0;
}