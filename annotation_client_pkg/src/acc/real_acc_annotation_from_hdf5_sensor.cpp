#include <annotation_client_pkg/annotation_client.hpp>


AnnotationClient::AnnotationClient(ros::NodeHandle &nh):
    nh_(nh),
    pnh_("~")
{
    set_paramenter();
    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
    mesh_client_ = nh_.serviceClient<common_srvs::MeshCloudService>(mesh_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
    hdf5_record_client_ = nh_.serviceClient<common_srvs::Hdf5RecordAcc>(hdf5_record_service_name_);
    tf_br_client_ = nh_.serviceClient<common_srvs::TfBroadcastService>(tf_br_service_name_);
    hdf5_open_client_ = nh_.serviceClient<common_srvs::Hdf5OpenSensorDataService>(hdf5_open_acc_service_name_);
}

void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    nearest_radious_ = param_list["nearest_radious"];
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    mesh_service_name_ = static_cast<std::string>(param_list["mesh_service_name"]);
    hdf5_record_service_name_ = static_cast<std::string>(param_list["hdf5_record_service_name"]);
    hdf5_open_acc_service_name_ = static_cast<std::string>(param_list["hdf5_open_acc_service_name"]);
    tf_br_service_name_ = static_cast<std::string>(param_list["tf_br_service_name"]);
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    qxyz_step_ = param_list["qxyz_step"];
    xyz_step_ = param_list["xyz_step"];
    object_list_.push_back(static_cast<std::string>(param_list["main_object_name"]));
    hdf5_record_file_path_ = static_cast<std::string>(param_list["hdf5_record_file_path"]);
    hdf5_open_file_path_ = static_cast<std::string>(param_list["hdf5_open_file_path"]);
}

void AnnotationClient::main()
{
    common_srvs::VisualizeCloud visual_srv;
    common_srvs::Hdf5OpenSensorDataService hdf5_open_sensor_srv;
    int data_size;
    int i = 0;
    while (1) {
        hdf5_open_sensor_srv.request.index = i;
        hdf5_open_sensor_srv.request.hdf5_open_file_path = hdf5_open_file_path_;
        Util::client_request(hdf5_open_client_, hdf5_open_sensor_srv, hdf5_open_acc_service_name_);
        visual_srv.request.cloud_data_list.push_back(hdf5_open_sensor_srv.response.cloud_data);
        visual_srv.request.topic_name_list.push_back("index_" + std::to_string(i));
        Util::client_request(visualize_client_, visual_srv, visualize_service_name_);
        nh_.getParam("hdf5_data_size", data_size);
        i++;
        if (i >= data_size) {
            break;
        }
    }
    int the_number_of_object, data_index;
    
    ROS_INFO_STREAM("What index of hdf5 data do you want to get?");
    std::cin >> data_index;
    
    hdf5_open_sensor_srv.request.index = data_index;
    Util::client_request(hdf5_open_client_, hdf5_open_sensor_srv, hdf5_open_acc_service_name_);
    pcl::PointCloud<PclXyz> pcl_data = UtilMsgData::cloudmsg_to_pcl(hdf5_open_sensor_srv.response.cloud_data);
    cloud_process_.set_crop_frame(sensor_frame_, world_frame_);
    pcl_data = cloud_process_.cropbox_segmenter(pcl_data);
    common_msgs::CloudData ano_data = UtilMsgData::pcl_to_cloudmsg(pcl_data), ano_copy_data;
    ano_copy_data = ano_data;
    visual_srv.request.cloud_data_list.push_back(ano_data);
    visual_srv.request.topic_name_list.push_back("ano_copy_data");
    Util::client_request(visualize_client_, visual_srv, visualize_service_name_);
    ROS_INFO_STREAM("How many object do you set?");
    std::cin >> the_number_of_object;
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
        zero_trans.rotation = TfFunction::make_geo_quaternion(TfFunction::rotate_xyz_make(0, 0, 0));
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
            ros::Duration(0.1);
            common_msgs::ObjectInfo mesh_input;
            mesh_input.object_name = object_list_[0];
            mesh_input.tf_name = tf_name_list[i];
            common_srvs::MeshCloudService mesh_srv;
            mesh_srv.request.multi_object_info.push_back(mesh_input);
            Util::client_request(mesh_client_, mesh_srv, mesh_service_name_);
            ano_copy_data = InstanceLabelDrawer::extract_nearest_point(ano_copy_data, mesh_srv.response.mesh[0], i+1, nearest_radious_);
            common_srvs::VisualizeCloud visual_srv1;
            visual_srv1.request.cloud_data_list.push_back(ano_copy_data);
            visual_srv1.request.topic_name_list.push_back("ano_copy_data");
            visual_srv1.request.cloud_data_list.push_back(mesh_srv.response.mesh[0]);
            visual_srv1.request.topic_name_list.push_back("mesh_cloud");
            Util::client_request(visualize_client_, visual_srv1, visualize_service_name_);
            if (key_tf.quit) {
                ano_data = ano_copy_data;
                pose_list[i] = mesh_srv.response.pose[0];
                break;
            }
        }
    }
    common_srvs::Hdf5RecordAcc record_srv;
    record_srv.request.index = 0;
    record_srv.request.record_file_path = hdf5_record_file_path_;
    record_srv.request.camera_info = hdf5_open_sensor_srv.response.camera_info;
    record_srv.request.image = hdf5_open_sensor_srv.response.image;
    record_srv.request.pose_data_list = pose_list;
    record_srv.request.cloud_data = ano_data;
    record_srv.request.the_number_of_dataset = the_number_of_dataset_;
    Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
}