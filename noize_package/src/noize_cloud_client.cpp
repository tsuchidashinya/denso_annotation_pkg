#include <noize_package/noize_client.hpp>

NoizeClient::NoizeClient(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_parameter();
    hdf5_open_client_ = nh_.serviceClient<common_srvs::Hdf5OpenSegmentationService>(hdf5_open_acc_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
}

void NoizeClient::set_parameter()
{
    pnh_.getParam("noize_client", param_list);
    hdf5_open_acc_service_name_ = static_cast<std::string>(param_list["hdf5_open_acc_service_name"]);
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    hdf5_open_file_path_ = static_cast<std::string>(param_list["hdf5_open_file_path"]);
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
}

void NoizeClient::main()
{
    // int data_size;
    // nh_.getParam("hdf5_data_size", data_size);
    // for (int i = 1; i <= data_size; i++) {
    //     common_srvs::Hdf5OpenSegmentationService hdf5_open_srv;
    //     hdf5_open_srv.request.index = i;
    //     Util::client_request(hdf5_open_client_, hdf5_open_srv, hdf5_open_acc_service_name_);
    //     common_srvs::VisualizeCloud visual_srv;
    //     visual_srv.request.cloud_data_list.push_back(hdf5_open_srv.response.cloud_data);
    //     visual_srv.request.topic_name_list.push_back("index_" + std::to_string(i));
    //     Util::client_request(visualize_client_, visual_srv, visualize_service_name_);
    // }
    NoizeCloudMake noize;
    common_msgs::CloudData noize_cloud, sum_cloud, noize_add;
    geometry_msgs::Transform transl;
    // noize_cloud = noize.sphere_unit(0.01, 1000);
    noize_cloud = noize.cylinder_unit(0.002, 0.005, 30);
    for (int i = 0; i < 1000; i++) {
        geometry_msgs::Vector3 translation;
        float x = 0.2;
        float y = 0.001;
        float z = 0.3;
        translation.x = util_.random_float(-x, x);
        translation.y = util_.random_float(-y, y);
        translation.z = util_.random_float(-z, z);
        noize_add = NoizeCloudTransform::translation_noize(noize_cloud, translation);
        sum_cloud = UtilMsgData::concat_cloudmsg(sum_cloud, noize_add);
    }
    sum_cloud = UtilMsgData::pclLabel_to_cloudmsg(CloudProcess::downsample_by_voxelgrid(UtilMsgData::cloudmsg_to_pclLabel(sum_cloud), 0.0015));
    sum_cloud.frame_id = world_frame_;
    common_srvs::VisualizeCloud visual_srv;
    visual_srv.request.cloud_data_list.push_back(sum_cloud);
    visual_srv.request.topic_name_list.push_back("noize_cloud");
    auto rotate_quat = TfFunction::rotate_xyz_make(M_PI/4, 0, 0);
    sum_cloud = NoizeCloudTransform::rotate_noize(sum_cloud, TfFunction::tf2_quat_to_geo_quat(rotate_quat));
    visual_srv.request.cloud_data_list.push_back(sum_cloud);
    visual_srv.request.topic_name_list.push_back("noize_cloud_rotate");
    Util::client_request(visualize_client_, visual_srv, visualize_service_name_);
}