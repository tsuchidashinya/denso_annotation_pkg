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
    common_srvs::Hdf5OpenAccService hdf5_open_srv;
    int data_size;
    int i = 0;
    while (ros::ok()) {
        hdf5_open_srv.request.index = i;
        hdf5_open_srv.request.hdf5_open_file_path = hdf5_open_file_path_;
        Util::client_request(hdf5_open_client_, hdf5_open_srv, hdf5_open_acc_service_name_);
        visual_srv.request.cloud_data_list.push_back(hdf5_open_srv.response.cloud_data);
        visual_srv.request.topic_name_list.push_back("index_" + std::to_string(i));
        Util::client_request(visualize_client_, visual_srv, visualize_service_name_);
        nh_.getParam("hdf5_data_size", data_size);
        i++;
        if (i >= data_size) {
            break;
        }
    }

    while (ros::ok()) {
        int data_index;
        ROS_INFO_STREAM("What index of hdf5 data do you want to get?");
        std::cin >> data_index;
        hdf5_open_srv.request.index = data_index;
        Util::client_request(hdf5_open_client_, hdf5_open_srv, hdf5_open_acc_service_name_);
        common_msgs::CloudData ano_data = hdf5_open_srv.response.cloud_data, ano_copy_data, ano_visual_data;
        ano_copy_data = ano_data;
        visual_srv.request.cloud_data_list.push_back(crop_cloudmsg(ano_data));
        visual_srv.request.topic_name_list.push_back("ano_visual_data");
        Util::client_request(visualize_client_, visual_srv, visualize_service_name_);
        std::vector<std::string> tf_name_list;
        std::vector<int> ins_list;
        std::vector<geometry_msgs::Transform> tf_transform_list;
        std::vector<common_msgs::PoseData> pose_list, pose_hdf5_open_list;
        pose_list = hdf5_open_srv.response.pose_data;
        for (int i = 0; i < pose_list.size(); i++) {
            geometry_msgs::TransformStamped trans_stamp;
            geometry_msgs::Transform trans_ori, trans_add;
            trans_ori = UtilMsgData::posedata_to_transform(pose_list[i]);
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
            ins_list.push_back(i + 1);
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
                ano_data = UtilMsgData::change_ins_cloudmsg(ano_data, index + 1, 0);
                geometry_msgs::TransformStamped final_tf;
                geometry_msgs::Transform zero_trans;
                zero_trans.rotation = TfFunction::make_geo_quaternion(TfFunction::rotate_xyz_make(0, 0, 0));
                final_tf = TfFunction::make_geo_trans_stamped(tf_name, world_frame_, tf_transform_list[index]);
                while (ros::ok())
                {
                    ano_visual_data = ano_copy_data;
                    common_srvs::VisualizeCloud visual_srv2;
                    visual_srv2.request.cloud_data_list.push_back(crop_cloudmsg(ano_visual_data));
                    visual_srv2.request.topic_name_list.push_back("ano_visual_data");
                    Util::client_request(visualize_client_, visual_srv2, visualize_service_name_);
                    ano_copy_data = ano_data;
                    KeyBoardTf key_tf = tf_func_.get_keyboard_tf(xyz_step_, qxyz_step_);
                    final_tf.transform = tf_func_.add_keyboard_tf(final_tf.transform, key_tf);
                    common_srvs::TfBroadcastService tf_br_srv;
                    tf_br_srv.request.broadcast_tf = final_tf;
                    tf_br_srv.request.tf_name = final_tf.child_frame_id;
                    Util::client_request(tf_br_client_, tf_br_srv, tf_br_service_name_);
                    ros::Duration(0.09);
                    common_msgs::ObjectInfo mesh_input;
                    mesh_input.object_name = object_list_[0];
                    mesh_input.tf_name = tf_name;
                    common_srvs::MeshCloudService mesh_srv;
                    mesh_srv.request.multi_object_info.push_back(mesh_input);
                    Util::client_request(mesh_client_, mesh_srv, mesh_service_name_);
                    ano_copy_data = InstanceLabelDrawer::extract_nearest_point(ano_copy_data, mesh_srv.response.mesh[0], index+1, nearest_radious_);
                    common_srvs::VisualizeCloud visual_srv1;
                    visual_srv1.request.cloud_data_list.push_back(crop_cloudmsg(ano_visual_data));
                    visual_srv1.request.topic_name_list.push_back("ano_visual_data");
                    visual_srv1.request.cloud_data_list.push_back(mesh_srv.response.mesh[0]);
                    visual_srv1.request.topic_name_list.push_back("mesh_cloud");
                    Util::client_request(visualize_client_, visual_srv1, visualize_service_name_);
                    if (key_tf.quit) {
                        ano_data = ano_copy_data;
                        Util::message_show("pose_list_size" + std::to_string(index), pose_list.size());
                        pose_list[index] = mesh_srv.response.pose[0];
                        pose_list[index].instance = ins_list[index];
                        break;
                    }
                }
                
            }
            else if (key_input == "d") {
                ;
            }
            else {
                ROS_ERROR_STREAM("Key input error!! Please a or d or q");
            }
            
        }
        for (int i = 0; i < tf_name_list.size(); i++) {
            common_msgs::ObjectInfo mesh_input1;
            mesh_input1.object_name = object_list_[0];
            mesh_input1.tf_name = tf_name_list[i];
            common_srvs::MeshCloudService mesh_srv1;
            mesh_srv1.request.multi_object_info.push_back(mesh_input1);
            Util::client_request(mesh_client_, mesh_srv1, mesh_service_name_);
            ano_data = InstanceLabelDrawer::extract_nearest_point(ano_data, mesh_srv1.response.mesh[0], ins_list[i], nearest_radious_);
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
        
    }
    
}