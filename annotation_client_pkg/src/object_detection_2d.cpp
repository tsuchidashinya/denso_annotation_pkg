#include <annotation_client_pkg/annotation_client.hpp>

void AnnotationClient::main()
{
    DecidePosition decide_gazebo_object;
    GazeboModelMove gazebo_model_move(nh_);
    std::vector<common_msgs::ObjectInfo> multi_object, multi_object_first;
    common_msgs::ObjectInfo sensor_object;
    sensor_object = decide_gazebo_object.get_sensor_position();
    gazebo_model_move.set_gazebo_model(sensor_object);
    ros::Duration(2.0);
    for (int i = 0; i < util_.random_int(1, 24); i++) {
        common_msgs::ObjectInfo object;
        object = decide_gazebo_object.make_object_info(i, "HV8");
        multi_object.push_back(object);
    }
    multi_object_first = multi_object;
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
    img = Make2DInfoBy3D::draw_b_box(img, box_pos);
    cv::resize(img, img, cv::Size(), 0.7, 0.7) ;
    cv::imshow("window", img);
    cv::waitKey(1000);
    multi_object_first = decide_gazebo_object.get_remove_position(multi_object_first);
    gazebo_model_move.set_multi_gazebo_model(multi_object_first);
}

