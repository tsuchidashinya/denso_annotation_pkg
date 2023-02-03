#include <annotation_client_pkg/annotation_client.hpp>


void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    nearest_radious_ = param_list["nearest_radious"];
    visualize_service_name_ = "visualize_cloud_service";
    vis_delete_service_name_ = "visualize_delete_service";
    sensor_service_name_ = "sensor_service";
    mesh_service_name_ = "mesh_service";
    hdf5_record_service_name_ = "record_segmentation_service";
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    gazebo_sensor_service_name_ = static_cast<std::string>(param_list["gazebo_sensor_service_name"]);
    occlusion_object_radious_ = param_list["occlusion_object_radious"];
    object_list_.push_back(static_cast<std::string>(param_list["main_object_name"]));
    hdf5_record_file_path_ = static_cast<std::string>(param_list["hdf5_record_file_path"]);
    quantity_of_object_list_.push_back(static_cast<int>(param_list["quantity_of_main_object"]));
    instance_of_object_list_.push_back(static_cast<int>(param_list["instance_of_main_object"]));
    pnh_.getParam("annotation_main/other_object_list", param_list);
    for (int i = 0; i < param_list.size(); i++) {
        object_list_.push_back(static_cast<std::string>(param_list[i]["object_name"]));
        quantity_of_object_list_.push_back(static_cast<int>(param_list[i]["quantity_of_the_object"]));
        instance_of_object_list_.push_back(static_cast<int>(param_list[i]["instance_of_the_object"]));
    }

    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
    mesh_client_ = nh_.serviceClient<common_srvs::MeshCloudService>(mesh_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
    vis_img_client_ = nh_.serviceClient<common_srvs::VisualizeImage>(vis_img_service_name_);
    vis_delete_client_ = nh_.serviceClient<common_srvs::VisualizeDeleteService>(vis_delete_service_name_);
    hdf5_record_client_ = nh_.serviceClient<common_srvs::Hdf5RecordSegmentation>(hdf5_record_service_name_);
    hdf5_record_2_client_ = nh_.serviceClient<common_srvs::Hdf5RecordSensorData>(hdf5_record_2_service_name_);
    tf_br_client_ = nh_.serviceClient<common_srvs::TfBroadcastService>(tf_br_service_name_);
    tf_delete_client_ = nh_.serviceClient<common_srvs::TfDeleteService>(tf_delete_service_name_);
    hdf5_open_client_ = nh_.serviceClient<common_srvs::Hdf5OpenAccService>(hdf5_open_acc_service_name_);
}

bool AnnotationClient::main()
{
    GazeboMoveServer gazebo_model_move(nh_);
    common_srvs::VisualizeCloud visualize_srv;
    common_srvs::VisualizeDeleteService vis_delete_srv;

    common_msgs::ObjectInfo sensor_object;
    sensor_object = decide_gazebo_object_.get_sensor_position();
    gazebo_model_move.set_gazebo_model(sensor_object);
    
    std::vector<common_msgs::ObjectInfo> multi_object, multi_object_all;
    for (int i = 0; i < object_list_.size(); i++) {
        for (int j = 0; j < quantity_of_object_list_[i]; j++) {
            common_msgs::ObjectInfo object;
            object = decide_gazebo_object_.make_object_info(j, object_list_[i]);
            multi_object_all.push_back(object);
            vis_delete_srv.request.delete_cloud_topic_list.push_back(object.tf_name + "_visualize");
        }
    }
    multi_object_all = decide_gazebo_object_.get_remove_position(multi_object_all);
    gazebo_model_move.set_multi_gazebo_model(multi_object_all);
    for (int i = 0; i < util_.random_int(0, quantity_of_object_list_[0]); i++) {
        common_msgs::ObjectInfo object;
        object = decide_gazebo_object_.make_object_info(i, object_list_[0]);
        multi_object.push_back(object);
    }
    multi_object = decide_gazebo_object_.get_randam_place_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    ros::Duration(0.7).sleep();
    
    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 1;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    common_msgs::CloudData sensor_cloud = sensor_srv.response.cloud_data;
    cv::Mat img = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    std::vector<float> cinfo_list = UtilMsgData::caminfo_to_floatlist(sensor_srv.response.camera_info);
    multi_object = instance_drawer_.extract_occuluder(multi_object, occlusion_object_radious_);
    multi_object = Util::delete_empty_object_info(multi_object);
    Data3Dto2D make_2d_3d(cinfo_list, Util::get_image_size(img));
    std::vector<common_msgs::BoxPosition> box_pos = make_2d_3d.get_out_data(multi_object);
    // if (util_.random_float(0, 1) < 0.1) {
    //     gazebo_model_move.set_multi_gazebo_model(multi_object_all);
    // }
    common_srvs::MeshCloudService mesh_srv;
    mesh_srv.request.multi_object_info = multi_object;
    Util::client_request(mesh_client_, mesh_srv, mesh_service_name_);
    std::vector<common_msgs::CloudData> mesh_cloud_list = mesh_srv.response.mesh;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    sensor_cloud = sensor_srv.response.cloud_data;
    sensor_cloud = UtilMsgData::draw_all_ins_cloudmsg(sensor_cloud, 0);
    Data2Dto3D get3d(cinfo_list, Util::get_image_size(img));
    std::vector<common_msgs::CloudData> cloud_multi;

    cloud_multi = get3d.get_out_data(sensor_cloud, box_pos);
    for (int i = 0; i < cloud_multi.size(); i++) {
        int mesh_index = Util::find_tfname_from_cloudlist(mesh_cloud_list, cloud_multi[i].tf_name);
        int object_index = Util::find_element_vector(object_list_, cloud_multi[i].object_name);
        cloud_multi[i] = SpaceHandlingLibrary::search_nearest_point(cloud_multi[i], mesh_cloud_list[mesh_index], instance_of_object_list_[object_index], 0.002);
    }

    common_msgs::CloudData final_cloud;
    for (int i = 0; i < cloud_multi.size(); i++) {
        common_srvs::Hdf5RecordSegmentation record_srv;
        record_srv.request.record_file_path = hdf5_record_file_path_;
        record_srv.request.cloud_data = cloud_multi[i];
        record_srv.request.the_number_of_dataset = the_number_of_dataset_;
        Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
        if (record_srv.response.finish) {
            return true;
        }
        visualize_srv.request.cloud_data_list.push_back(cloud_multi[i]);
        visualize_srv.request.topic_name_list.push_back(cloud_multi[i].tf_name + "_visualize");
        final_cloud = UtilMsgData::concat_cloudmsg(final_cloud, cloud_multi[i]);
        ros::Duration(0.01).sleep();
    }
    Util::client_request(vis_delete_client_, vis_delete_srv, vis_delete_service_name_);
    visualize_srv.request.cloud_data_list.push_back(final_cloud);
    visualize_srv.request.topic_name_list.push_back("final_cloud");
    Util::client_request(visualize_client_, visualize_srv, visualize_service_name_);
    return false;
}