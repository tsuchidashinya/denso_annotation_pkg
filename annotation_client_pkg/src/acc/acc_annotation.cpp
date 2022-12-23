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
}

void AnnotationClient::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("annotation_main", param_list);
    nearest_radious_ = param_list["nearest_radious"];
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    sensor_service_name_ = static_cast<std::string>(param_list["sensor_service_name"]);
    mesh_service_name_ = static_cast<std::string>(param_list["mesh_service_name"]);
    hdf5_record_service_name_ = static_cast<std::string>(param_list["hdf5_record_service_name"]);
    the_number_of_dataset_ = param_list["the_number_of_dataset"];
    gazebo_sensor_service_name_ = static_cast<std::string>(param_list["gazebo_sensor_service_name"]);
}

void AnnotationClient::acc_main(int count)
{
    DecidePosition decide_gazebo_object;
    GazeboMoveServer gazebo_model_move(nh_);

    // common_msgs::ObjectInfo sensor_pos_info = decide_gazebo_object.get_sensor_position();
    // gazebo_model_move.set_gazebo_model(sensor_pos_info);
    common_msgs::ObjectInfo sensor_object;
    sensor_object = decide_gazebo_object.get_sensor_position();
    gazebo_model_move.set_gazebo_model(sensor_object);
    
    std::vector<common_msgs::ObjectInfo> multi_object, multi_object_all;
    for (int i = 0; i < 30; i++) {
        common_msgs::ObjectInfo object;
        object = decide_gazebo_object.make_object_info(i, "HV8");
        multi_object_all.push_back(object);
    }
    multi_object_all = decide_gazebo_object.get_remove_position(multi_object_all);
    gazebo_model_move.set_multi_gazebo_model(multi_object_all);
    for (int i = 0; i < util_.random_int(1, 24); i++) {
        common_msgs::ObjectInfo object;
        object = decide_gazebo_object.make_object_info(i, "HV8");
        multi_object.push_back(object);
    }
    multi_object = decide_gazebo_object.get_randam_place_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    ros::Duration(0.5).sleep();
    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 1;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    common_msgs::CloudData sensor_cloud = sensor_srv.response.cloud_data;
    cv::Mat img = UtilMsgData::rosimg_to_cvimg(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    std::vector<float> cinfo_list = UtilMsgData::caminfo_to_floatlist(sensor_srv.response.camera_info);
    multi_object = instance_drawer_.extract_occuluder(multi_object, 0.038);
    multi_object = Util::delete_empty_object_info(multi_object);
    // Data3Dto2D make_2d_3d(cinfo_list, Util::get_image_size(img));
    // std::vector<common_msgs::BoxPosition> box_pos = make_2d_3d.get_out_data(UtilAnno::tf_listen_frames_from_objectinfo(multi_object));
    // for (int i = 0; i < box_pos.size(); i++) {
    //     YoloFormat yolo_data = UtilMsgData::pascalvoc_to_yolo(box_pos[i], Util::get_image_size(img));
    //     if (util_.random_float(0, 1) < 0.03) {
    //         float scale_up = util_.random_float(1, 3);
    //         yolo_data.w = scale_up * yolo_data.w;
    //         yolo_data.h = scale_up * yolo_data.h;
    //     }
    //     if (util_.random_float(0, 1) < 0.005) {
    //         do  {
    //             yolo_data.x = yolo_data.x * util_.random_float(0.1, 1.5);
    //         } while (yolo_data.x > 1);
    //         do  {
    //             yolo_data.y = yolo_data.y * util_.random_float(0.1, 1.5);
    //         } while (yolo_data.y > 1);
    //     }
        
    //     box_pos[i] = UtilMsgData::yolo_to_pascalvoc(yolo_data, Util::get_image_size(img));
    // }
    common_srvs::MeshCloudService mesh_srv;
    mesh_srv.request.multi_object_info = multi_object;
    Util::client_request(mesh_client_, mesh_srv, mesh_service_name_);
    std::vector<common_msgs::CloudData> mesh_cloud_list = mesh_srv.response.mesh;

    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    sensor_cloud = sensor_srv.response.cloud_data;
    sensor_cloud = InstanceLabelDrawer::draw_instance_all(sensor_cloud, 0);
    for (int i = 0; i < mesh_cloud_list.size(); i++) {
        sensor_cloud = InstanceLabelDrawer::extract_nearest_point(sensor_cloud, mesh_cloud_list[i], 1, 0.002);
    }
    // Data2Dto3D get3d(cinfo_list, Util::get_image_size(img));
    // std::vector<common_msgs::CloudData> cloud_multi = get3d.get_out_data(sensor_cloud, box_pos);
    // std::vector<std::string> topic_list;
    // for (int i = 0; i < cloud_multi.size(); i++) {
    //     common_srvs::Hdf5RecordSegmentation record_srv;
    //     record_srv.request.cloud_data = cloud_multi[i];
    //     record_srv.request.the_number_of_dataset = the_number_of_dataset_;
    //     Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
    //     topic_list.push_back(cloud_multi[i].cloud_name + "cloud_multi");
    //     ros::Duration(0.1).sleep();
    // }
    common_srvs::Hdf5RecordAcc record_srv;
    record_srv.request.camera_info = cinfo_list;
    record_srv.request.image = sensor_srv.response.image;
    record_srv.request.pose_data_list = mesh_srv.response.pose;
    record_srv.request.cloud_data = sensor_cloud;
    if (count >= the_number_of_dataset_ - 1) {
        record_srv.request.is_end = 1;
    }
    else {
        record_srv.request.is_end = 0;
    }
    Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);

    common_srvs::VisualizeCloud visualize_srv;
    visualize_srv.request.cloud_data_list.push_back(sensor_cloud);
    visualize_srv.request.topic_name_list.push_back("acc_cloud");
    Util::client_request(visualize_client_, visualize_srv, visualize_service_name_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_segmentation");
    ros::NodeHandle nh;
    AnnotationClient annotation_main(nh);
    int counter;
    for (int i = 0; i < annotation_main.the_number_of_dataset_; i++) {
        annotation_main.acc_main(i);
        // ros::Duration(0.1).sleep();
        Util::message_show("Progress rate", std::to_string(i + 1) + "/" + std::to_string(annotation_main.the_number_of_dataset_));
        nh.getParam("record_counter", counter);
        if (counter >= annotation_main.the_number_of_dataset_) {
            break;
        }
    }
    return 0;
}