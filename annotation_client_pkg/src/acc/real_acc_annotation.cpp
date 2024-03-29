#include <annotation_client_pkg/annotation_client.hpp>


void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    nearest_radious_ = param_list["nearest_radious"];
    visualize_service_name_ = "visualize_cloud_service";
    sensor_service_name_ = "sensor_service";
    mesh_service_name_ = "mesh_service";
    hdf5_record_service_name_ = "record_acc_service";
    tf_br_service_name_ = "tf_broadcast_service";
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    qxyz_step_ = param_list["qxyz_step"];
    xyz_step_ = param_list["xyz_step"];
    ObjectListType object_option;
    object_option.object_name = static_cast<std::string>(param_list["main_object_name"]);
    object_option_list_.push_back(object_option);
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
    int the_number_of_object;
    ROS_INFO_STREAM("How many object do you set?");
    std::cin >> the_number_of_object;
    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 0;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    common_msgs::CloudData ano_data = sensor_srv.response.cloud_data, ano_copy_data;
    ano_copy_data = ano_data;
    std::vector<std::string> tf_name_list;
    tf_name_list.resize(the_number_of_object);
    std::vector<common_msgs::PoseData> pose_list;
    pose_list.resize(the_number_of_object);
    for (int i = 0; i < the_number_of_object; i++) {
        ROS_INFO_STREAM("What tf name you move?");
        bool is_same_element_exist = false;
        do {
            std::cin >> tf_name_list[i];
            is_same_element_exist = Util::is_same_element_exist(tf_name_list, i);
        } while (is_same_element_exist);
        geometry_msgs::TransformStamped final_tf;
        geometry_msgs::Transform zero_trans;
        zero_trans.rotation = TfFunction::tf2_quat_to_geo_quat(TfFunction::rotate_xyz_make(0, 0, 0));
        final_tf = TfFunction::make_geo_trans_stamped(tf_name_list[i], world_frame_, zero_trans);
        while (ros::ok())
        {
            ano_copy_data = ano_data;
            KeyBoardTf key_tf = tf_func_.get_keyboard_tf(xyz_step_, qxyz_step_);
            final_tf.transform = tf_func_.add_keyboard_tf(final_tf.transform, key_tf);
            common_srvs::TfBroadcastService tf_br_srv;
            tf_br_srv.request.broadcast_tf = final_tf;
            tf_br_srv.request.tf_name = final_tf.child_frame_id;
            Util::client_request(tf_br_client_, tf_br_srv, tf_br_service_name_);
            ros::Duration(0.09);
            common_msgs::ObjectInfo mesh_input;
            mesh_input.object_name = object_option_list_[0].object_name;
            mesh_input.tf_name = tf_name_list[i];
            common_srvs::MeshCloudService mesh_srv;
            mesh_srv.request.multi_object_info.push_back(mesh_input);
            Util::client_request(mesh_client_, mesh_srv, mesh_service_name_);
            ano_copy_data = SpaceHandlingLibrary::search_nearest_point(ano_copy_data, mesh_srv.response.mesh[0], i+1, nearest_radious_);
            common_srvs::VisualizeCloud visual_srv;
            visual_srv.request.cloud_data_list.push_back(ano_copy_data);
            visual_srv.request.topic_name_list.push_back("ano_copy_data");
            visual_srv.request.cloud_data_list.push_back(mesh_srv.response.mesh[0]);
            visual_srv.request.topic_name_list.push_back("mesh_data");
            Util::client_request(visualize_client_, visual_srv, visualize_service_name_);
            if (key_tf.quit) {
                ano_data = ano_copy_data;
                pose_list[i] = mesh_srv.response.pose[0];
                break;
            }
        }
    }
    common_srvs::Hdf5RecordAcc record_srv;
    record_srv.request.record_file_path = hdf5_record_file_path_;
    record_srv.request.index = 0;
    record_srv.request.camera_info = UtilMsgData::caminfo_to_floatlist(sensor_srv.response.camera_info);
    record_srv.request.image = sensor_srv.response.image;
    record_srv.request.pose_data_list = pose_list;
    record_srv.request.cloud_data = ano_data;
    record_srv.request.is_end = 1;
    Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
    return false;
}