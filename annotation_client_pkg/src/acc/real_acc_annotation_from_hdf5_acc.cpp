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
    hdf5_open_client_ = nh_.serviceClient<common_srvs::Hdf5OpenService>(hdf5_open_service_name_);
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
    hdf5_open_service_name_ = static_cast<std::string>(param_list["hdf5_open_service_name"]);
    tf_br_service_name_ = static_cast<std::string>(param_list["tf_br_service_name"]);
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    qxyz_step_ = param_list["qxyz_step"];
    xyz_step_ = param_list["xyz_step"];
    object_list_.push_back(static_cast<std::string>(param_list["main_object_name"]));
}

void AnnotationClient::main()
{
    int the_number_of_object, data_index;
    ROS_INFO_STREAM("What index of hdf5 data do you want to get?");
    std::cin >> data_index;
    common_srvs::Hdf5OpenService hdf5_open_srv;
    hdf5_open_srv.request.index = data_index;
    Util::client_request(hdf5_open_client_, hdf5_open_srv, hdf5_open_service_name_);
    common_msgs::CloudData ano_data = hdf5_open_srv.response.cloud_data, ano_copy_data, ano_visual_data;
    ano_copy_data = ano_data;
    std::vector<std::string> tf_name_list;
    std::vector<geometry_msgs::Transform> tf_transform_list;
    std::vector<common_msgs::PoseData> pose_list, pose_hdf5_open_list;
    pose_hdf5_open_list = hdf5_open_srv.response.pose_data;
    for (int i = 0; i < pose_hdf5_open_list.size(); i++) {
        geometry_msgs::TransformStamped trans_stamp;
        geometry_msgs::Transform trans_ori, trans_add;
        trans_ori = UtilMsgData::posedata_to_transform(pose_hdf5_open_list[i]);
        trans_add = tf_func_.tf_listen(sensor_frame_, world_frame_);
        trans_stamp.transform = TfFunction::change_tf_frame_by_rotate(trans_ori, trans_add);
        trans_stamp.header.frame_id = world_frame_;
        trans_stamp.child_frame_id = "tf_" + std::to_string(i);
        common_srvs::TfBroadcastService tf_srv;
        tf_srv.request.broadcast_tf = trans_stamp;
        tf_srv.request.tf_name = trans_stamp.child_frame_id;
        tf_name_list.push_back(trans_stamp.child_frame_id);
        tf_transform_list.push_back(trans_stamp.transform);
        Util::client_request(tf_br_client_, tf_srv, tf_br_service_name_);
    }
    int index = 0;
    std::string key_tf_name;
    bool quit = false;
    while (1) {
        ROS_INFO_STREAM("Do you quit?(Yes is y or Y, No is other key");
        std::string input;
        std::cin >> input;
        if (input == "y" || input == "Y") {
            break;
        }
        ROS_INFO_STREAM("What tf name you move?");
        bool is_same_element_exist = false;
        std::cin >> key_tf_name;
        index = Util::find_element_vector(tf_name_list, key_tf_name);
        if (index == -1) {
            tf_name_list.push_back(key_tf_name);
            geometry_msgs::Transform zero_trans;
            zero_trans.rotation = TfFunction::make_geo_quaternion(TfFunction::rotate_xyz_make(0, 0, 0));
            tf_transform_list.push_back(zero_trans);
            index = tf_name_list.size() - 1;
            common_msgs::PoseData pose;
            pose_list.push_back(pose);
        }
        geometry_msgs::TransformStamped final_tf;
        geometry_msgs::Transform zero_trans;
        zero_trans.rotation = TfFunction::make_geo_quaternion(TfFunction::rotate_xyz_make(0, 0, 0));
        final_tf = TfFunction::make_geo_trans_stamped(key_tf_name, world_frame_, tf_transform_list[index]);
        while (ros::ok())
        {
            ano_visual_data = ano_copy_data;
            common_srvs::VisualizeCloud visual_srv;
            visual_srv.request.cloud_data_list.push_back(ano_visual_data);
            visual_srv.request.topic_name_list.push_back("ano_visual_data");
            Util::client_request(visualize_client_, visual_srv, visualize_service_name_);
            ano_copy_data = ano_data;
            KeyBoardTf key_tf = tf_func_.get_keyboard_tf(xyz_step_, qxyz_step_);
            final_tf.transform = tf_func_.add_keyboard_tf(final_tf.transform, key_tf);
            common_srvs::TfBroadcastService tf_br_srv;
            tf_br_srv.request.broadcast_tf = final_tf;
            tf_br_srv.request.tf_name = final_tf.child_frame_id;
            Util::client_request(tf_br_client_, tf_br_srv, tf_br_service_name_);
            common_msgs::ObjectInfo mesh_input;
            mesh_input.object_name = object_list_[0];
            mesh_input.tf_name = key_tf_name;
            common_srvs::MeshCloudService mesh_srv;
            mesh_srv.request.multi_object_info.push_back(mesh_input);
            Util::client_request(mesh_client_, mesh_srv, mesh_service_name_);
            ano_copy_data = InstanceLabelDrawer::extract_nearest_point(ano_copy_data, mesh_srv.response.mesh[0], index+1, nearest_radious_);
            
            if (key_tf.quit) {
                ano_data = ano_copy_data;
                pose_list[index] = mesh_srv.response.pose[0];
                break;
            }
        }
    }
    common_srvs::Hdf5RecordAcc record_srv;
    record_srv.request.camera_info = hdf5_open_srv.response.camera_info;
    record_srv.request.image = hdf5_open_srv.response.image;
    record_srv.request.pose_data_list = pose_list;
    record_srv.request.cloud_data = ano_data;
    record_srv.request.the_number_of_dataset = the_number_of_dataset_;
    Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
}