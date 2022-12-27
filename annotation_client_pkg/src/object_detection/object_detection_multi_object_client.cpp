#include <annotation_client_pkg/annotation_client.hpp>


void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    nearest_radious_ = param_list["nearest_radious"];
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    sensor_service_name_ = static_cast<std::string>(param_list["sensor_service_name"]);
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    vis_img_service_name_ = static_cast<std::string>(param_list["visualize_image_service_name"]);
    save_base_file_name_ = static_cast<std::string>(param_list["save_base_file_name"]);
    save_dir_ = static_cast<std::string>(param_list["save_dir"]);
    save_dir_ = Util::join(save_dir_, save_base_file_name_);
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
    DecidePosition decide_gazebo_object;
    GazeboMoveServer gazebo_model_move(nh_);
    common_srvs::VisualizeImage vis_img_srv;
    
    common_msgs::ObjectInfo sensor_object;
    sensor_object = decide_gazebo_object.get_sensor_position();
    gazebo_model_move.set_gazebo_model(sensor_object);

    std::vector<common_msgs::ObjectInfo> multi_object, multi_object_all, multi_object_other_kind;
    for (int i = 0; i < object_list_.size(); i++) {
        for (int j = 0; j < quantity_of_object_list_[i]; j++) {
            common_msgs::ObjectInfo object;
            object = decide_gazebo_object.make_object_info(j, object_list_[i]);
            multi_object_all.push_back(object);
        }
    }
    multi_object_all = decide_gazebo_object.get_remove_position(multi_object_all);
    gazebo_model_move.set_multi_gazebo_model(multi_object_all);
    for (int i = 0; i < util_.random_int(1, quantity_of_object_list_[0]); i++) {
        common_msgs::ObjectInfo object;
        object = decide_gazebo_object.make_object_info(i, object_list_[0]);
        multi_object.push_back(object);
    }
    multi_object = decide_gazebo_object.get_randam_place_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    if (util_.random_float(0, 1) < 0.15) {
        for (int i = 1; i < object_list_.size(); i++) {
            for (int j = 0; j < util_.random_int(1, quantity_of_object_list_[i]); j++) {
                common_msgs::ObjectInfo object;
                object = decide_gazebo_object.make_object_info(j, object_list_[i]);
                multi_object_other_kind.push_back(object);
            }
        }
    }
    multi_object_other_kind = decide_gazebo_object.get_randam_place_position(multi_object_other_kind);
    gazebo_model_move.set_multi_gazebo_model(multi_object_other_kind);
    ros::Duration(0.2).sleep();
    

    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 1;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    cv::Mat img_ori, img;
    img = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    img_ori = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    std::vector<float> cinfo_list = UtilMsgData::caminfo_to_floatlist(sensor_srv.response.camera_info);
    multi_object = instance_drawer_.extract_occuluder(multi_object, occlusion_object_radious_);
    Data3Dto2D make_2d_3d(cinfo_list, Util::get_image_size(img));
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
    box_pos = UtilMsgData::set_classname_on_boxposition(box_pos, "HV8_occuluder");
    for (int i = 0; i < box_pos.size(); i++) {
        box_pos[i] = UtilMsgData::box_position_normalized(box_pos[i]);
    }
    Util::write_b_box_label(box_pos, Util::join(label_dir_path, final_base_file_name + ".txt"));
    vis_img_srv.request.image_list.push_back(UtilMsgData::cvimg_to_rosimg(img_ori, sensor_msgs::image_encodings::BGR8));
    vis_img_srv.request.topic_name_list.push_back("original_image");
    vis_img_srv.request.image_list.push_back(UtilMsgData::cvimg_to_rosimg(img, sensor_msgs::image_encodings::BGR8));
    vis_img_srv.request.topic_name_list.push_back("box_draw_image");
    Util::client_request(vis_img_client_, vis_img_srv, vis_img_service_name_);
}

