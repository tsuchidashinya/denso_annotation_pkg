#include <annotation_client_pkg/semantic_segmentation_client.hpp>
#include <opencv2/opencv.hpp>

SemanticSegmentation::SemanticSegmentation(ros::NodeHandle &nh):
    nh_(nh),
    pnh_("~")
{
    set_paramenter();
    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
    mesh_client_ = nh_.serviceClient<anno_srvs::MeshCloudService>(mesh_service_name_);
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pc_pub_topic_, 1);
}

void SemanticSegmentation::set_paramenter()
{
    pnh_.getParam("common_parameter", param_list);
    sensor_service_name_ = static_cast<std::string>(param_list["sensor_service_name"]);
    mesh_service_name_ = static_cast<std::string>(param_list["mesh_service_name"]);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pc_visualize_data_.header.frame_id = sensor_frame_;
    pnh_.getParam("annotation_main", param_list);
    pc_pub_topic_ = static_cast<std::string>(param_list["visualize_pc_topic_name"]);
    nearest_radious_ = param_list["nearest_radious"];
}

void SemanticSegmentation::main()
{
    DecidePosition decide_gazebo_object;
    GazeboModelMove gazebo_model_move(nh_);

    // anno_msgs::ObjectInfo sensor_pos_info = decide_gazebo_object.get_sensor_position();
    // gazebo_model_move.set_gazebo_model(sensor_pos_info);

    std::vector<anno_msgs::ObjectInfo> multi_object;
    for (int i = 0; i < 10; i++) {
        anno_msgs::ObjectInfo object;
        object = decide_gazebo_object.make_object_info(i, "HV8");
        multi_object.push_back(object);
    }
    multi_object = decide_gazebo_object.get_remove_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    multi_object = decide_gazebo_object.get_randam_place_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    ros::Duration(0.5).sleep();
    // anno_srvs::MeshCloudService mesh_srv;
    // mesh_srv.request.multi_object_info = multi_object;
    // UtilBase::client_request(mesh_client_, mesh_srv, mesh_service_name_);
    // std::vector<common_msgs::CloudData> mesh_clouds = mesh_srv.response.mesh;

    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 1;
    UtilBase::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    common_msgs::CloudData sensor_cloud = sensor_srv.response.cloud_data;
    cv::Mat img = UtilSensor::img_to_cv(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    Make2DInfoBy3D make_2d_3d(sensor_srv.response.camera_info, FuncDataConvertion::get_image_size(img));
    multi_object = instance_drawer_.extract_occuluder(multi_object, 0.04);
    UtilBase::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    sensor_cloud = sensor_srv.response.cloud_data;
    multi_object = instance_drawer_.extract_occuluder(multi_object, 0.04);
    std::vector<common_msgs::BoxPosition> box_pos = make_2d_3d.get_out_data(UtilAnno::get_tf_frames_from_objectinfo(multi_object));
    // img = Make2DInfoBy3D::draw_b_box(img, box_pos);
    // cv::resize(img, img, cv::Size(), 0.7, 0.7) ;
    // cv::imshow("window", img);
    // cv::waitKey(1000);

    Get3DBy2D get3d(sensor_srv.response.camera_info, FuncDataConvertion::get_image_size(img));
    std::vector<common_msgs::CloudData> cloud_multi = get3d.get_out_data(sensor_cloud, box_pos);
    
    // cloud_multi = instance_drawer_.detect_occuluder(cloud_multi, 1, 20, 0.01);
    
    common_msgs::CloudData sum_cloud;
    for (int i = 0; i < cloud_multi.size(); i++) {
        cloud_multi[i] = InstanceLabelDrawer::draw_instance_all(cloud_multi[i], i+1);
        sum_cloud = UtilSensor::concat_cloudmsg(sum_cloud, cloud_multi[i]);
    }
    sum_cloud = UtilSensor::remove_ins_cloudmsg(sum_cloud, 20);
    pcl::PointCloud<PclRgb> pclrgb = util_sensor_.cloudmsg_to_pclrgb(sum_cloud);
    pc_visualize_data_ = util_sensor_.pclrgb_to_pc2_color(pclrgb);
    // InstanceLabelDrawer::draw_initial_instance(sensor_srv.response.cloud_data, 0);
    // sensor_cloud = InstanceLabelDrawer::extract_nearest_point(sensor_cloud, mesh_clouds[1], 2, nearest_radious_);
    // sensor_cloud = InstanceLabelDrawer::extract_nearest_point(sensor_cloud, mesh_clouds[0], 1, nearest_radious_);
    // pcl::PointCloud<PclRgb> pclrgb = util_sensor_.cloudmsg_to_pclrgb(sensor_cloud);
    // pc_visualize_data_ = util_sensor_.pclrgb_to_pc2_color(pclrgb);
}

void SemanticSegmentation::visualize_publish()
{
    pc_visualize_data_.header.frame_id = sensor_frame_;
    pc_pub_.publish(pc_visualize_data_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_segmentation");
    ros::NodeHandle nh;
    SemanticSegmentation semseg(nh);
    for (int i = 0; i < 10; i++) {
        semseg.main();
        semseg.visualize_publish();
        ros::Duration(1).sleep();
    }
    
    return 0;
}