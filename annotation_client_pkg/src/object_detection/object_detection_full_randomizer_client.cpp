#include <annotation_client_pkg/annotation_client.hpp>


void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    nearest_radious_ = param_list["nearest_radious"];
    vis_img_service_name_ = "visualize_image_service";
    sensor_service_name_ = "sensor_service";
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    save_base_file_name_ = static_cast<std::string>(param_list["save_base_file_name"]);
    save_dir_ = static_cast<std::string>(param_list["save_dir"]);
    ObjectListType object_option;
    object_option.instance_num = static_cast<int>(param_list["instance_of_main_object"]);
    object_option.num_of_object = static_cast<int>(param_list["num_of_main_object"]);
    object_option.object_name = static_cast<std::string>(param_list["main_object_name"]);
    object_option.radious = param_list["radious_of_main_object"];
    object_option.occlusion_radious = param_list["occlusion_radious_of_main_object"];
    object_option_list_.push_back(object_option);
    pnh_.getParam("annotation_main/other_object_list", param_list);
    for (int i = 0; i < param_list.size(); i++) {
        ObjectListType other_object_option;
        other_object_option.instance_num = static_cast<int>(param_list[i]["instance_of_the_object"]);
        other_object_option.num_of_object = static_cast<int>(param_list[i]["num_of_object"]);
        other_object_option.object_name = static_cast<std::string>(param_list[i]["object_name"]);
        other_object_option.occlusion_radious = param_list[i]["occlusion_radious"];
        other_object_option.radious = param_list[i]["radious"];
        object_option_list_.push_back(other_object_option);
    }

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
    std::string final_base_file_name = save_base_file_name_ + "_" + Util::get_time_str();
    NoizeImageClient noize_image_client(nh_);
    GazeboMoveServer gazebo_model_move(nh_);
    common_srvs::VisualizeImage vis_img_srv;
    noize_image_client.restore_image();
    if (util_.probability() < 0.5) {
        noize_image_client.noize_image_main();
    }
    auto light_para = decide_gazebo_object_.get_light_properties();
    gazebo_model_move.set_light(light_para);
    auto link_para = decide_gazebo_object_.get_link_visual_parameter("denso_box");
    gazebo_model_move.set_link_visual(link_para);
    //  if (util_.probability() < 1) {
    //     noize_image_client.noize_image_main(Util::join("/home/ericlab/Pictures/domain_texture", final_base_file_name + ".jpg"));
    // }
    
    common_msgs::ObjectInfo sensor_object;
    sensor_object = decide_gazebo_object_.get_sensor_position();
    gazebo_model_move.set_gazebo_model(sensor_object);

    std::vector<common_msgs::ObjectInfo> multi_object, multi_object_all;
    int object_counter = 0;
    for (int i = 0; i < object_option_list_.size(); i++) {
        for (int j = 0; j < object_option_list_[i].num_of_object; j++) {
            common_msgs::ObjectInfo object;
            object = DecidePosition::make_object_info(object_counter, j, object_option_list_[i]);
            multi_object_all.push_back(object);
            object_counter++;
        }
    }
    multi_object_all = decide_gazebo_object_.get_remove_position(multi_object_all);
    gazebo_model_move.set_multi_gazebo_model(multi_object_all);
    object_counter = 0;
    for (int i = 0; i < util_.random_int(1, object_option_list_[0].num_of_object); i++) {
        common_msgs::ObjectInfo object;
        object = decide_gazebo_object_.make_object_info(object_counter, i, object_option_list_[0]);
        multi_object.push_back(object);
        object_counter++;
    }
    // for (int i = 0; i < util_.random_int(1, quantity_of_object_list_[1]); i++) {
    //     common_msgs::ObjectInfo object;
    //     object = decide_gazebo_object_.make_object_info(i, object_list_[1]);
    //     multi_object.push_back(object);
    // }
    multi_object = decide_gazebo_object_.get_randam_place_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    
    ros::Duration(0.7).sleep();
    

    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 1;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    cv::Mat img_ori, img;
    img = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    img_ori = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    std::vector<float> cinfo_list = UtilMsgData::caminfo_to_floatlist(sensor_srv.response.camera_info);
    multi_object = instance_drawer_.extract_occuluder(multi_object);
    Data3Dto2D draw_3d_to_2d(cinfo_list, Util::get_image_size(img));
    std::vector<common_msgs::BoxPosition> box_pos = draw_3d_to_2d.get_out_data(multi_object);
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
    
    cv::imwrite(Util::join(image_dir_path, final_base_file_name + ".jpg"), img_ori);
    cv::imwrite(Util::join(box_dir_path, final_base_file_name + ".jpg"), img);
    for (int i = 0; i < box_pos.size(); i++) {
        box_pos[i] = UtilMsgData::box_position_normalized(box_pos[i]);
    }
    Util::write_b_box_label(box_pos, Util::join(label_dir_path, final_base_file_name + ".txt"));
    vis_img_srv.request.image_list.push_back(UtilMsgData::cvimg_to_rosimg(img_ori, sensor_msgs::image_encodings::BGR8));
    vis_img_srv.request.topic_name_list.push_back("original_image");
    vis_img_srv.request.image_list.push_back(UtilMsgData::cvimg_to_rosimg(img, sensor_msgs::image_encodings::BGR8));
    vis_img_srv.request.topic_name_list.push_back("box_draw_image");
    Util::client_request(vis_img_client_, vis_img_srv, vis_img_service_name_);
    return false;
}

