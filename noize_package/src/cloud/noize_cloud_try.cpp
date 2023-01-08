#include <noize_package/cloud/noize_cloud_client.hpp>

NoizeCloudClient::NoizeCloudClient(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_parameter();
    hdf5_open_client_ = nh_.serviceClient<common_srvs::Hdf5OpenSegmentationService>(hdf5_open_acc_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
}

void NoizeCloudClient::set_parameter()
{
    pnh_.getParam("noize_client", param_list);
    hdf5_open_acc_service_name_ = static_cast<std::string>(param_list["hdf5_open_acc_service_name"]);
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    hdf5_open_file_path_ = static_cast<std::string>(param_list["hdf5_open_file_path"]);
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
}

void NoizeCloudClient::visualize_request(std::string topic_name, common_msgs::CloudData cloud)
{
    common_srvs::VisualizeCloud vis_srv;
    vis_srv.request.topic_name_list.push_back(topic_name);
    vis_srv.request.cloud_data_list.push_back(cloud);
    Util::client_request(visualize_client_, vis_srv, visualize_service_name_);
    Util::message_show("visualize_request", topic_name);
}

void NoizeCloudClient::main()
{
    int index = 0;
    std::vector<common_msgs::CloudData> original_cloud_list;
    while (1) {
        common_srvs::Hdf5OpenSegmentationService hdf5_srv;
        hdf5_srv.request.index = index;
        hdf5_srv.request.hdf5_open_file_path = hdf5_open_file_path_;
        // Util::message_show("index", index);
        Util::client_request(hdf5_open_client_, hdf5_srv, hdf5_open_acc_service_name_);
        common_msgs::CloudData cloud_data = hdf5_srv.response.cloud_data;
        original_cloud_list.push_back(cloud_data);
        // visualize_request("index_" + std::to_string(index), cloud_data);
        if (index >= hdf5_srv.response.data_size - 1) {
            break;
        }
        index++;
    }
    for (int all_index = 0; all_index < original_cloud_list.size(); all_index++) {
        auto origin_1 = UtilMsgData::extract_ins_cloudmsg(original_cloud_list[all_index], 1);
        auto centroid = NoizeCloudTransform::get_centroid(origin_1);
        auto far_away_cloud = NoizeCloudTransform::get_far_away_cloud(origin_1, centroid, 1);
        NoizeCloudMake noize;
        auto board_cloud = noize.rectangle_cloud(0.05, util_.random_float(0.02, 0.05), 0.001, util_.random_int(2000, 10000));
        // board_cloud.frame_id = world_frame_;
        auto translation = UtilMsgData::cloudmsg_to_vector3(far_away_cloud);
        auto trans_cloud = UtilMsgData::vector3_to_cloudmsg(translation);
        // visualize_request("trans_cloud", trans_cloud);
        translation = NoizeCloudTransform::change_frame_id(translation, sensor_frame_, world_frame_);
        auto trans_cloud_after = UtilMsgData::vector3_to_cloudmsg(translation);
        trans_cloud_after.frame_id = world_frame_;
        auto final_cloud = original_cloud_list[all_index];
        common_msgs::CloudData sum_cloud;
        // sum_cloud = noize.noize_cloud_random(0.03, 0.03, 0.05);
        for (int i = 0; i < 4; i++) {
            auto noize_part =  noize.circle_cloud(util_.random_float(0.03, 0.3), util_.random_float(0, M_PI * 3/ 2), util_.random_float(0.0001, 0.003), Util::random_int_static(100, 1500));
            auto quaternion = TfFunction::rotate_xyz_make(util_.random_float(-M_PI, M_PI), util_.random_float(-M_PI, M_PI), util_.random_float(-M_PI, M_PI));
            noize_part = NoizeCloudTransform::rotate_noize(noize_part, quaternion);
            sum_cloud = UtilMsgData::concat_cloudmsg(sum_cloud, noize_part);
        }
        for (int i = 0; i < 4; i++) {
            auto noize_part =  noize.linear_cloud(util_.random_float(0.03, 0.3), util_.random_float(0.0001, 0.003), Util::random_int_static(100, 1500));
            auto quaternion = TfFunction::rotate_xyz_make(util_.random_float(-M_PI, M_PI), util_.random_float(-M_PI, M_PI), util_.random_float(-M_PI, M_PI));
            noize_part = NoizeCloudTransform::rotate_noize(noize_part, quaternion);
            sum_cloud = UtilMsgData::concat_cloudmsg(sum_cloud, noize_part);
        }
        
        sum_cloud.frame_id = world_frame_;
        visualize_request("sum_cloud" + std::to_string(all_index), sum_cloud);
        auto sum_cloud_copy = sum_cloud;
        auto sample_num = util_.random_int(10, 1500);
        if (sample_num < sum_cloud.x.size()) {
            auto pcl_cloud = UtilMsgData::cloudmsg_to_pclLabel(sum_cloud);
            pcl_cloud = CloudProcess::downsample_random(pcl_cloud, sample_num);
            sum_cloud = UtilMsgData::pclLabel_to_cloudmsg(pcl_cloud);
        }
        // visualize_request("noize_cloud_rotate", sum_cloud);
        centroid = NoizeCloudTransform::change_frame_id(centroid, sensor_frame_, world_frame_);
        sum_cloud = NoizeCloudTransform::translation_noize(sum_cloud, centroid);
        auto noize_cloud_final = NoizeCloudTransform::change_frame_id(sum_cloud, world_frame_, sensor_frame_);
        sum_cloud_copy = NoizeCloudTransform::translation_noize(sum_cloud_copy, centroid);
        auto noize_cloud_final_not_down = NoizeCloudTransform::change_frame_id(sum_cloud_copy, world_frame_, sensor_frame_);
        final_cloud = UtilMsgData::concat_cloudmsg(final_cloud, noize_cloud_final);
        visualize_request("final_cloud" + std::to_string(all_index), final_cloud);
    }
    
}