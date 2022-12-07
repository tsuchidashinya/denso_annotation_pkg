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
    save_base_file_name_ = static_cast<std::string>(param_list["save_base_file_name"]);
    save_dir_ = static_cast<std::string>(param_list["save_dir"]);
    save_dir_ = Util::join(save_dir_, save_base_file_name_);
    occlusion_object_radious_ = param_list["occlusion_object_radious"];
}

void AnnotationClient::main()
{
    DecidePosition decide_gazebo_object;
    GazeboMoveServer gazebo_model_move(nh_);
    std::vector<common_msgs::ObjectInfo> multi_object, multi_object_all;
    for (int i = 0; i < 30; i++) {
        common_msgs::ObjectInfo object;
        object = decide_gazebo_object.make_object_info(i, "HV8");
        multi_object_all.push_back(object);
    }
    multi_object_all = decide_gazebo_object.get_remove_position(multi_object_all);
    gazebo_model_move.set_multi_gazebo_model(multi_object_all);
    common_msgs::ObjectInfo sensor_object;
    sensor_object = decide_gazebo_object.get_sensor_position();
    gazebo_model_move.set_gazebo_model(sensor_object);
    ros::Duration(1.5);
    for (int i = 0; i < util_.random_int(1, 24); i++) {
        common_msgs::ObjectInfo object;
        object = decide_gazebo_object.make_object_info(i, "HV8");
        multi_object.push_back(object);
    }
    multi_object = decide_gazebo_object.get_randam_place_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    ros::Duration(1).sleep();
    

    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 1;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    common_msgs::CloudData sensor_cloud = sensor_srv.response.cloud_data;
    cv::Mat img_ori, img = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    img_ori = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    std::vector<float> cinfo_list = UtilMsgData::caminfo_to_floatlist(sensor_srv.response.camera_info);
    Data3Dto2D make_2d_3d(cinfo_list, Util::get_image_size(img));
    multi_object = instance_drawer_.extract_occuluder(multi_object, occlusion_object_radious_);
    // multi_object = instance_drawer_.extract_occuluder(multi_object, occlusion_object_radious_);
    std::vector<common_msgs::BoxPosition> box_pos = make_2d_3d.get_out_data(multi_object);
    for (int i = 0; i < box_pos.size(); i++) {
        YoloFormat yolo_data = UtilMsgData::pascalvoc_to_yolo(box_pos[i], Util::get_image_size(img));
        
        float scale_up = 1.2;
        yolo_data.w = scale_up * yolo_data.w;
        yolo_data.h = scale_up * yolo_data.h;
        
        box_pos[i] = UtilMsgData::yolo_to_pascalvoc(yolo_data, Util::get_image_size(img));
    }
    img = Data3Dto2D::draw_b_box(img, box_pos);
    std::string image_dir_path = Util::join(save_dir_, "images");
    std::string box_dir_path = Util::join(save_dir_, "boxes");
    std::string label_dir_path = Util::join(save_dir_, "labels");
    Util::mkdir(image_dir_path);
    Util::mkdir(box_dir_path);
    Util::mkdir(label_dir_path);
    std::string final_base_file_name = save_base_file_name_ + "_" + Util::get_time_str();
    cv::imwrite(Util::join(image_dir_path, final_base_file_name + ".jpg"), img_ori);
    cv::imwrite(Util::join(box_dir_path, final_base_file_name + ".jpg"), img);
    box_pos = InstanceLabelDrawer::set_object_class_name(box_pos, "HV8_occuluder");
    for (int i = 0; i < box_pos.size(); i++) {
        box_pos[i] = UtilMsgData::box_position_normalized(box_pos[i]);
    }
    Util::write_b_box_label(box_pos, Util::join(label_dir_path, final_base_file_name + ".txt"));
}

