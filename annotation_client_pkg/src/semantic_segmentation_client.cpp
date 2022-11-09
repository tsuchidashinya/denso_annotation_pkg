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
    pnh_.getParam("annotation_main", param_list);
    sensor_service_name_ = static_cast<std::string>(param_list["sensor_service_name"]);
    mesh_service_name_ = static_cast<std::string>(param_list["mesh_service_name"]);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pc_visualize_data_.header.frame_id = sensor_frame_;
    pc_pub_topic_ = static_cast<std::string>(param_list["visualize_pc_topic_name"]);
    nearest_radious_ = param_list["nearest_radious"];
}

void SemanticSegmentation::main()
{
    DecidePosition decide_gazebo_object;
    GazeboModelMove gazebo_model_move(nh_);

    anno_msgs::ObjectInfo sensor_pos_info = decide_gazebo_object.get_sensor_position();
    gazebo_model_move.set_gazebo_model(sensor_pos_info);
    geometry_msgs::TransformStamped sensor_tf;
    geometry_msgs::Transform sensor_trans;
    sensor_trans.translation = sensor_pos_info.position.translation;
    sensor_trans.rotation = TfBasic::make_geo_quaternion(TfBasic::rotate_xyz_make(0, M_PI/2, 0, TfBasic::make_tf2_quaternion(sensor_pos_info.position.rotation)));
    sensor_tf = TfBasic::make_geo_trans_stamped("photoneo_center", world_frame_, sensor_trans);
    tf_basic_.broadcast(sensor_tf);

    std::vector<anno_msgs::ObjectInfo> multi_object;
    for (int i = 0; i < 10; i++) {
        anno_msgs::ObjectInfo object;
        object = decide_gazebo_object.make_object_info(i, "HV8");
        multi_object.push_back(object);
    }

    multi_object = decide_gazebo_object.get_randam_place_position(multi_object);
    gazebo_model_move.set_multi_gazebo_model(multi_object);
    ros::Duration(0.3).sleep();
    anno_srvs::MeshCloudService mesh_srv;
    mesh_srv.request.multi_object_info = multi_object;
    UtilBase::client_request(mesh_client_, mesh_srv, mesh_service_name_);
    std::vector<common_msgs::CloudData> mesh_clouds = mesh_srv.response.mesh;

    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 1;
    UtilBase::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    common_msgs::CloudData sensor_cloud = sensor_srv.response.cloud_data;
    tf_basic_.broadcast(sensor_tf);
    cv::Mat img = UtilSensor::img_to_cv(sensor_srv.response.image, sensor_msgs::image_encodings::BGR8);
    Make2DInfoBy3D make_2d_3d(sensor_srv.response.camera_info, FuncDataConvertion::get_image_size(img));
    
    std::vector<common_msgs::BoxPosition> box_pos = make_2d_3d.get_out_data(UtilAnno::get_tf_frames_from_objectinfo(multi_object));
    UtilBase::message_show("box_size", box_pos.size());
    img = Make2DInfoBy3D::draw_b_box(img, box_pos);
    // cv::resize(img, img, cv::Size(), 0.7, 0.7) ;
    // cv::imshow("window", img);
    // cv::waitKey(3000);
    NearestPointExtractor::draw_initial_instance(sensor_srv.response.cloud_data, 0);
    sensor_cloud = NearestPointExtractor::extract_nearest_point(sensor_cloud, mesh_clouds[1], 2, nearest_radious_);
    sensor_cloud = NearestPointExtractor::extract_nearest_point(sensor_cloud, mesh_clouds[0], 1, nearest_radious_);
    pcl::PointCloud<PclRgb> pclrgb = util_sensor_.cloudmsg_to_pclrgb(sensor_cloud);
    pc_visualize_data_ = util_sensor_.pclrgb_to_pc2_color(pclrgb);
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
    for (int i = 0; i < 20; i++) {
        semseg.main();
        semseg.visualize_publish();
        ros::Duration(3).sleep();
    }
    
    return 0;
}