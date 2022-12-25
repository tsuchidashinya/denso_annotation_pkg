#include <annotation_client_pkg/annotation_client.hpp>


AnnotationClient::AnnotationClient(ros::NodeHandle &nh):
    nh_(nh),
    pnh_("~")
{
    set_paramenter();
    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
    mesh_client_ = nh_.serviceClient<common_srvs::MeshCloudService>(mesh_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
    vis_delete_client_ = nh_.serviceClient<common_srvs::VisualizeDeleteService>(vis_delete_service_name_);
    hdf5_record_client_ = nh_.serviceClient<common_srvs::Hdf5RecordAcc>(hdf5_record_service_name_);
    tf_br_client_ = nh_.serviceClient<common_srvs::TfBroadcastService>(tf_br_service_name_);
    tf_delete_client_ = nh_.serviceClient<common_srvs::TfDeleteService>(tf_delete_service_name_);
    hdf5_open_client_ = nh_.serviceClient<common_srvs::Hdf5OpenAccService>(hdf5_open_acc_service_name_);
}

void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    nearest_radious_ = param_list["nearest_radious"];
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    vis_delete_service_name_ = static_cast<std::string>(param_list["visualize_delete_service_name"]);
    mesh_service_name_ = static_cast<std::string>(param_list["mesh_service_name"]);
    hdf5_record_service_name_ = static_cast<std::string>(param_list["hdf5_record_service_name"]);
    hdf5_open_acc_service_name_ = static_cast<std::string>(param_list["hdf5_open_acc_service_name"]);
    tf_br_service_name_ = static_cast<std::string>(param_list["tf_br_service_name"]);
    tf_delete_service_name_ = static_cast<std::string>(param_list["tf_delete_service_name"]);
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    qxyz_step_ = param_list["qxyz_step"];
    xyz_step_ = param_list["xyz_step"];
    object_list_.push_back(static_cast<std::string>(param_list["main_object_name"]));
    hdf5_record_file_path_ = static_cast<std::string>(param_list["hdf5_record_file_path"]);
    hdf5_open_file_path_ = static_cast<std::string>(param_list["hdf5_open_file_path"]);
}



void AnnotationClient::main()
{
    common_srvs::Hdf5OpenAccService hdf5_open_srv;
    common_srvs::MeshCloudService mesh_srv;
    common_srvs::VisualizeDeleteService vis_delete_srv;
    std::vector<int> hdf5_open_index_list;
    std::string get_hdf5_file_path = hdf5_open_file_path_;
    while (ros::ok()) {
        int i = 0;
        bool reload = true;
        while (ros::ok()) {
            hdf5_open_srv.request.index = i;
            hdf5_open_srv.request.hdf5_open_file_path = get_hdf5_file_path;
            hdf5_open_srv.request.is_reload = reload;
            Util::client_request(hdf5_open_client_, hdf5_open_srv, hdf5_open_acc_service_name_);
            reload = false;
            vis_delete_srv.request.delete_cloud_topic_list.push_back("index_" + std::to_string(i));
            visualize_request(crop_cloudmsg(hdf5_open_srv.response.cloud_data), "index_" + std::to_string(i));
            int data_size;
            nh_.getParam("hdf5_data_size", data_size);
            hdf5_open_index_list.push_back(i);
            i++;
            if (i >= data_size) {
                break;
            }
        }
        int data_index;
        while (ros::ok()) {
            ROS_WARN_STREAM("Please look at the topic \"index_*\"");
            ROS_INFO_STREAM("What index of hdf5 data do you want to get?");
            std::cin >> data_index;
            int find_index = Util::find_element_vector(hdf5_open_index_list, data_index);
            if (find_index != -1) {
                break;
            }
        }
        Util::client_request(vis_delete_client_, vis_delete_srv, vis_delete_service_name_);
        ROS_WARN_STREAM("Please look at the topic \"ano_visual_data\"");
        hdf5_open_srv.request.index = data_index;
        hdf5_open_srv.request.hdf5_open_file_path = get_hdf5_file_path;
        Util::client_request(hdf5_open_client_, hdf5_open_srv, hdf5_open_acc_service_name_);
        common_msgs::CloudData ano_data = hdf5_open_srv.response.cloud_data, ano_copy_data, ano_visual_data;
        ano_copy_data = ano_data;
        visualize_request(crop_cloudmsg(ano_data), "ano_visual_data");
        std::vector<std::string> tf_name_list;
        std::vector<int> ins_list;
        std::vector<geometry_msgs::Transform> tf_transform_list;
        std::vector<common_msgs::PoseData> pose_list;
        pose_list = hdf5_open_srv.response.pose_data;
        for (int i = 0; i < pose_list.size(); i++) {
            geometry_msgs::TransformStamped trans_stamp;
            geometry_msgs::Transform trans_ori, trans_add;
            trans_ori = UtilMsgData::posedata_to_transform(pose_list[i]);
            trans_add = tf_func_.tf_listen(sensor_frame_, world_frame_);
            trans_stamp.transform = TfFunction::change_tf_frame_by_rotate(trans_ori, trans_add);
            trans_stamp.header.frame_id = world_frame_;
            trans_stamp.child_frame_id = "tf_" + std::to_string(i);
            tf_broadcast_request(trans_stamp);
            tf_name_list.push_back(trans_stamp.child_frame_id);
            tf_transform_list.push_back(trans_stamp.transform);
            ins_list.push_back(pose_list[i].instance);
        }
        int index = 0;
        bool quit = false;
        while (ros::ok()) {
            ROS_INFO_STREAM("a) add tf   d) delete tf   q) quit");
            std::string key_input;
            std::cin >> key_input;
            if (key_input == "q") {
                break;
            }
            else if (key_input == "a") {
                ROS_INFO_STREAM("Please input \"tf_name ins_num\"");
                bool is_same_element_exist = false;
                std::string tf_name;
                int ins;
                std::cin >> tf_name >> ins;
                index = Util::find_element_vector(tf_name_list, tf_name);
                if (index == -1) {
                    tf_name_list.push_back(tf_name);
                    ins_list.push_back(ins);
                    geometry_msgs::Transform zero_trans;
                    zero_trans.rotation = TfFunction::make_geo_quaternion(TfFunction::rotate_xyz_make(0, 0, 0));
                    tf_transform_list.push_back(zero_trans);
                    index = tf_name_list.size() - 1;
                    common_msgs::PoseData pose;
                    pose_list.push_back(pose);
                }
                geometry_msgs::TransformStamped final_tf;
                final_tf = TfFunction::make_geo_trans_stamped(tf_name_list[index], world_frame_, tf_transform_list[index]);
                tf_broadcast_request(final_tf);
                ins_list[index] = ins;
                ano_data = InstanceLabelDrawer::draw_instance_all(ano_data, 0);
                for (int i = 0; i < tf_name_list.size(); i++) {
                    mesh_srv = mesh_request(tf_name_list[i]);
                    if (i == index) {
                        ano_data = nearest_extractor(ano_data, mesh_srv, 0);
                    }
                    else {
                        ano_data = nearest_extractor(ano_data, mesh_srv, ins_list[i]);
                    }
                }
                while (ros::ok())
                {
                    ano_copy_data = ano_data;
                    mesh_srv = mesh_request(tf_name_list[index]);
                    ano_copy_data = nearest_extractor(ano_copy_data, mesh_srv, ins_list[index]);
                    ano_visual_data = ano_copy_data;
                    visualize_request(crop_cloudmsg(ano_visual_data), "ano_visual_data");
                    visualize_request(mesh_srv.response.mesh[0], "mesh_cloud");
                    KeyBoardTf key_tf = tf_func_.get_keyboard_tf(xyz_step_, qxyz_step_);
                    final_tf.transform = tf_func_.add_keyboard_tf(final_tf.transform, key_tf);
                    tf_broadcast_request(final_tf);
                    ros::Duration(0.06);
                    if (key_tf.quit) {
                        ano_data = ano_copy_data;
                        pose_list[index] = mesh_srv.response.pose[0];
                        pose_list[index].instance = ins;
                        break;
                    }
                }
            }
            else if (key_input == "d") {
                ROS_INFO_STREAM("What tf you delete?");
                std::string delete_tf_name;
                std::cin >> delete_tf_name;
                int delete_index = Util::find_element_vector(tf_name_list, delete_tf_name);
                if (delete_index == -1) {
                    ;
                }
                else {
                    tf_name_list.erase(tf_name_list.begin() + delete_index);
                    pose_list.erase(pose_list.begin() + delete_index);
                    ins_list.erase(ins_list.begin() + delete_index);
                    common_srvs::TfDeleteService tf_delete_srv;
                    tf_delete_srv.request.delete_tf_name = delete_tf_name;
                    Util::client_request(tf_delete_client_, tf_delete_srv, tf_delete_service_name_);
                }
                ano_data = InstanceLabelDrawer::draw_instance_all(ano_data, 0);
                for (int i = 0; i < tf_name_list.size(); i++) {
                    mesh_srv = mesh_request(tf_name_list[i]);
                    ano_data = nearest_extractor(ano_data, mesh_srv, ins_list[i]);
                }
                visualize_request(crop_cloudmsg(ano_data), "ano_visual_data");
            }
            else {
                ROS_ERROR_STREAM("Key input error!! Please a or d or q");
            }
            
        }
        for (int i = 0; i < tf_name_list.size(); i++) {
            mesh_srv = mesh_request(tf_name_list[i]);
            ano_data = nearest_extractor(ano_data, mesh_srv, ins_list[i]);
        }
        for (int i = 0; i < pose_list.size(); i++) {
            pose_list[i].instance = ins_list[i];
        }
        common_srvs::Hdf5RecordAcc record_srv;
        record_srv.request.record_file_path = hdf5_record_file_path_;
        record_srv.request.index = data_index;
        record_srv.request.camera_info = hdf5_open_srv.response.camera_info;
        record_srv.request.image = hdf5_open_srv.response.image;
        record_srv.request.pose_data_list = pose_list;
        record_srv.request.cloud_data = ano_data;
        record_srv.request.is_overwrite = 1;
        std::string input;
        while (ros::ok()) {
            ROS_INFO_STREAM("Continue or quit (Continue is c or C, quit is q or Q");
            std::cin >> input;
            if (input == "q" || input=="Q" || input=="c" || input=="C") {
                break;
            }
            else {
                ROS_ERROR_STREAM("Key Error!!(Please c or C or q or Q)");
            }
        }
        if (input == "q" || input == "Q") {
            record_srv.request.is_end = 1;
            Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
            break;
        }
        else {
            record_srv.request.is_end = 0;
            Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
        }
        get_hdf5_file_path = record_srv.response.save_temp_file_path;
    }
    
}