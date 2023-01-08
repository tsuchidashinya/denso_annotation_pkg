#include <noize_package/cloud/noize_cloud_client.hpp>

NoizeCloudClient::NoizeCloudClient(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_parameter();
    hdf5_open_client_ = nh_.serviceClient<common_srvs::Hdf5OpenSegmentationService>(hdf5_open_acc_service_name_);
    hdf5_record_client_ = nh_.serviceClient<common_srvs::Hdf5RecordSegmentation>(hdf5_record_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
}

void NoizeCloudClient::set_parameter()
{
    pnh_.getParam("noize_client", param_list);
    hdf5_open_acc_service_name_ = static_cast<std::string>(param_list["hdf5_open_acc_service_name"]);
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
    hdf5_open_file_path_ = static_cast<std::string>(param_list["hdf5_open_file_path"]);
    hdf5_record_service_name_ = static_cast<std::string>(param_list["hdf5_record_service_name"]);
    hdf5_record_file_path_ = static_cast<std::string>(param_list["hdf5_record_file_path"]);
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
        // auto centroid_cloud = UtilMsgData::vector3_to_cloudmsg(centroid);
        // auto after_cloud = SpaceHandlingLibrary::search_nearest_point(original_cloud_list[all_index], centroid_cloud, 5, 0.02);
        // after_cloud = UtilMsgData::extract_ins_cloudmsg(after_cloud, 5);
        // visualize_request("after_cloud", after_cloud);
        auto far_away_cloud = NoizeCloudTransform::get_far_away_cloud(origin_1, centroid, 1);
        // visualize_request("far_away_cloud"+ std::to_string(all_index), far_away_cloud);
        // auto boarder_cloud = NoizeCloudTransform::get_instance_boarder_cloud(original_cloud_list[all_index], 0.004);
        // visualize_request("boarder_cloud", boarder_cloud);
        // auto defect_cloud = NoizeCloudMake::make_defect_cloud(original_cloud_list[all_index], after_cloud, 0.004);
        // visualize_request("defect_cloud", defect_cloud);
        NoizeCloudMake noize;
        auto board_cloud = noize.rectangle_cloud(0.05, util_.random_float(0.02, 0.05), 0.001, util_.random_int(2000, 10000));
        // board_cloud.frame_id = world_frame_;
        auto translation = UtilMsgData::cloudmsg_to_vector3(far_away_cloud);
        auto trans_cloud = UtilMsgData::vector3_to_cloudmsg(translation);
        // visualize_request("trans_cloud", trans_cloud);
        translation = NoizeCloudTransform::change_frame_id(translation, sensor_frame_, world_frame_);
        auto trans_cloud_after = UtilMsgData::vector3_to_cloudmsg(translation);
        trans_cloud_after.frame_id = world_frame_;
        // visualize_request("trans_cloud_after", trans_cloud_after);
        // auto rotate_quat = TfFunction::rotate_xyz_make(M_PI/2 + Util::random_float_static(-0.1, 0), 0, 0);
        // board_cloud = NoizeCloudTransform::rotate_noize(board_cloud, TfFunction::tf2_quat_to_geo_quat(rotate_quat));
        // auto noize_board = noize.noize_cloud_random(0.1, 0.1, 0.1);
        // for (int i = 0; i < util_.random_int(0, 10); i++) {
        //     auto noize_board_part = noize.noize_cloud_random(0.1, 0.1, 0.1);
        //     noize_board = UtilMsgData::concat_cloudmsg(noize_board, noize_board_part);
        // }
        auto final_cloud = original_cloud_list[all_index];
        // board_cloud = NoizeCloudMake::make_defect_cloud(board_cloud, noize_board, 0.005);
        // board_cloud = NoizeCloudTransform::translation_noize(board_cloud, translation);
        // board_cloud = NoizeCloudTransform::change_frame_id(board_cloud, world_frame_, sensor_frame_);
        
        // visualize_request("board_cloud" + std::to_string(all_index), board_cloud);
        common_msgs::CloudData sum_cloud;
        sum_cloud = noize.noize_cloud_random(0.03, 0.03, 0.05);
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
        // visualize_request("noize_cloud_final_not_down" + std::to_string(all_index), noize_cloud_final_not_down);
        // final_cloud = NoizeCloudMake::make_defect_cloud(final_cloud, noize_cloud_final, 0.0035);
        // visualize_request("noize_cloud_final" + std::to_string(all_index), noize_cloud_final);
        if (util_.probability() < 0.25) {
            final_cloud = UtilMsgData::concat_cloudmsg(final_cloud, noize_cloud_final);
        }
        // visualize_request("final_cloud" + std::to_string(all_index), final_cloud);
        common_srvs::Hdf5RecordSegmentation record_srv;
        record_srv.request.the_number_of_dataset = original_cloud_list.size();
        record_srv.request.record_file_path = hdf5_record_file_path_;
        record_srv.request.cloud_data = final_cloud;
        Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
    }
    
}