#include <annotation_client_pkg/annotation_client.hpp>

void AnnotationClient::main()
{
    DecidePosition decide_gazebo_object;
    GazeboModelMove gazebo_model_move(nh_);

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
    cv::Mat img = UtilMsgData::img_to_cv(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    std::vector<float> cinfo_list = UtilMsgData::caminfo_to_floatlist(sensor_srv.response.camera_info);
    Make2DInfoBy3D make_2d_3d(cinfo_list, UtilMsgData::get_image_size(img));
    multi_object = instance_drawer_.extract_occuluder(multi_object, 0.04);
    
    multi_object = instance_drawer_.extract_occuluder(multi_object, 0.04);
    std::vector<common_msgs::BoxPosition> box_pos = make_2d_3d.get_out_data(UtilAnno::tf_listen_frames_from_objectinfo(multi_object));
    // img = Make2DInfoBy3D::draw_b_box(img, box_pos);
    // cv::resize(img, img, cv::Size(), 0.7, 0.7) ;
    // cv::imshow("window", img);
    // cv::waitKey(1000);
    anno_srvs::MeshCloudService mesh_srv;
    mesh_srv.request.multi_object_info = multi_object;
    Util::client_request(mesh_client_, mesh_srv, mesh_service_name_);
    std::vector<common_msgs::CloudData> mesh_clouds = mesh_srv.response.mesh;

    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    sensor_cloud = sensor_srv.response.cloud_data;
    Get3DBy2D get3d(cinfo_list, UtilMsgData::get_image_size(img));
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
    // InstanceLabelDrawer::draw_initial_instance(sensor_srv.response.cloud_data, 0);
    // sensor_cloud = InstanceLabelDrawer::extract_nearest_point(sensor_cloud, mesh_clouds[1], 2, nearest_radious_);
    // sensor_cloud = InstanceLabelDrawer::extract_nearest_point(sensor_cloud, mesh_clouds[0], 1, nearest_radious_);
    // pcl::PointCloud<PclRgb> pclrgb = util_msg_data_.cloudmsg_to_pclrgb(sensor_cloud);
    // pc_visualize_data_ = util_msg_data_.pclrgb_to_pc2_color(pclrgb);
}


