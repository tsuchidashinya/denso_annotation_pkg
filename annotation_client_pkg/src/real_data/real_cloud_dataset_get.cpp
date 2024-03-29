#include <annotation_client_pkg/annotation_client.hpp>


void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    visualize_service_name_ = "visualize_cloud_service";
    sensor_service_name_ = "sensor_service";
    hdf5_record_service_name_ = "record_real_sensor_data_service";
    vis_img_service_name_ = "visualize_image_service";
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    hdf5_record_file_path_ = static_cast<std::string>(param_list["hdf5_record_file_path"]);

    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
    mesh_client_ = nh_.serviceClient<common_srvs::MeshCloudService>(mesh_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
    vis_img_client_ = nh_.serviceClient<common_srvs::VisualizeImage>(vis_img_service_name_);
    vis_delete_client_ = nh_.serviceClient<common_srvs::VisualizeDeleteService>(vis_delete_service_name_);
    hdf5_record_client_ = nh_.serviceClient<common_srvs::Hdf5RecordAcc>(hdf5_record_service_name_);
    hdf5_record_2_client_ = nh_.serviceClient<common_srvs::Hdf5RecordSensorData>(hdf5_record_2_service_name_);
    tf_br_client_ = nh_.serviceClient<common_srvs::TfBroadcastService>(tf_br_service_name_);
    tf_delete_client_ = nh_.serviceClient<common_srvs::TfDeleteService>(tf_delete_service_name_);
    hdf5_open_client_ = nh_.serviceClient<common_srvs::Hdf5OpenAccService>(hdf5_open_acc_service_name_);
}

bool AnnotationClient::main()
{
    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 1;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    common_msgs::CloudData sensor_cloud = sensor_srv.response.cloud_data;
    cv::Mat img = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    std::vector<float> cinfo_list = UtilMsgData::caminfo_to_floatlist(sensor_srv.response.camera_info);
    sensor_cloud = UtilMsgData::draw_all_ins_cloudmsg(sensor_cloud, 0);
    common_srvs::Hdf5RecordSensorData record_srv;
    record_srv.request.record_file_path = hdf5_record_file_path_;
    record_srv.request.camera_info = cinfo_list;
    record_srv.request.image = sensor_srv.response.image;
    record_srv.request.cloud_data = sensor_cloud;
    record_srv.request.is_end = the_number_of_dataset_;
    Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
    common_srvs::VisualizeCloud visualize_srv;
    visualize_srv.request.cloud_data_list.push_back(sensor_cloud);
    visualize_srv.request.topic_name_list.push_back("acc_cloud");
    Util::client_request(visualize_client_, visualize_srv, visualize_service_name_);
    common_srvs::VisualizeImage vis_img_srv;
    vis_img_srv.request.image_list.push_back(sensor_srv.response.image);
    vis_img_srv.request.topic_name_list.push_back("real_image");
    Util::client_request(vis_img_client_, vis_img_srv, vis_img_service_name_);
    return false;
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