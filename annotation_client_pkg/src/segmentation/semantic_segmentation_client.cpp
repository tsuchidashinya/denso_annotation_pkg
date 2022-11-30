#include <annotation_client_pkg/annotation_client.hpp>


AnnotationClient::AnnotationClient(ros::NodeHandle &nh):
    nh_(nh),
    pnh_("~")
{
    set_paramenter();
    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
    mesh_client_ = nh_.serviceClient<anno_srvs::MeshCloudService>(mesh_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
    record_client_ = nh_.serviceClient<anno_srvs::RecordSegmentation>(record_service_name_);
}

void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    nearest_radious_ = param_list["nearest_radious"];
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    sensor_service_name_ = static_cast<std::string>(param_list["sensor_service_name"]);
    mesh_service_name_ = static_cast<std::string>(param_list["mesh_service_name"]);
    record_service_name_ = static_cast<std::string>(param_list["record_service_name"]);
    the_number_of_dataset_ = param_list["the_number_of_dataset"];

}

void AnnotationClient::main()
{
    DecidePosition decide_gazebo_object;
    GazeboMoveServer gazebo_model_move(nh_);

    // common_msgs::ObjectInfo sensor_pos_info = decide_gazebo_object.get_sensor_position();
    // gazebo_model_move.set_gazebo_model(sensor_pos_info);

    std::vector<common_msgs::ObjectInfo> multi_object;
    for (int i = 0; i < 10; i++) {
        common_msgs::ObjectInfo object;
        object = decide_gazebo_object.make_object_info(i, "HV8");
        multi_object.push_back(object);
    }
    multi_object = decide_gazebo_object.get_remove_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    multi_object = decide_gazebo_object.get_randam_place_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    ros::Duration(0.5).sleep();
    

    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 1;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    common_msgs::CloudData sensor_cloud = sensor_srv.response.cloud_data;
    cv::Mat img = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    std::vector<float> cinfo_list = UtilMsgData::caminfo_to_floatlist(sensor_srv.response.camera_info);
    Data3Dto2D make_2d_3d(cinfo_list, Util::get_image_size(img));
    multi_object = instance_drawer_.extract_occuluder(multi_object, 0.04);
    
    multi_object = instance_drawer_.extract_occuluder(multi_object, 0.04);
    std::vector<common_msgs::BoxPosition> box_pos = make_2d_3d.get_out_data(UtilAnno::tf_listen_frames_from_objectinfo(multi_object));
    // img = Data3Dto2D::draw_b_box(img, box_pos);
    // cv::resize(img, img, cv::Size(), 0.7, 0.7) ;
    // cv::imshow("window", img);
    // cv::waitKey(1000);
    anno_srvs::MeshCloudService mesh_srv;
    mesh_srv.request.multi_object_info = multi_object;
    Util::client_request(mesh_client_, mesh_srv, mesh_service_name_);
    std::vector<common_msgs::CloudData> mesh_clouds = mesh_srv.response.mesh;

    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    sensor_cloud = sensor_srv.response.cloud_data;
    Data2Dto3D get3d(cinfo_list, Util::get_image_size(img));
    std::vector<common_msgs::CloudData> cloud_multi = get3d.get_out_data(sensor_cloud, box_pos);
    
    // cloud_multi = instance_drawer_.detect_occuluder(cloud_multi, 1, 20, 0.01);
    
    common_msgs::CloudData sum_cloud;
    for (int i = 0; i < cloud_multi.size(); i++) {
        cloud_multi[i] = InstanceLabelDrawer::draw_instance_all(cloud_multi[i], 0);
        cloud_multi[i] = InstanceLabelDrawer::extract_nearest_point(cloud_multi[i], mesh_clouds[i], 1, 0.002);
    }
    for (int i = 0; i < cloud_multi.size(); i++) {
        anno_srvs::RecordSegmentation record_srv;
        record_srv.request.cloud_data = cloud_multi[i];
        record_srv.request.the_number_of_dataset = the_number_of_dataset_;
        Util::client_request(record_client_, record_srv, record_service_name_);
        ros::Duration(0.1).sleep();
    }
    common_srvs::VisualizeCloud visualize_srv;
    visualize_srv.request.cloud_data_list = cloud_multi;
    Util::client_request(visualize_client_, visualize_srv, visualize_service_name_);
}