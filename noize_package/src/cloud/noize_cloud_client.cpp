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
    hdf5_open_acc_service_name_ = "hdf5_open_segmentation_service";
    visualize_service_name_ = "visualize_cloud_service";
    hdf5_open_file_path_ = static_cast<std::string>(param_list["hdf5_open_file_path"]);
    hdf5_record_service_name_ = "record_segmentation_service";
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
        Util::client_request(hdf5_open_client_, hdf5_srv, hdf5_open_acc_service_name_);
        common_msgs::CloudData cloud_data = hdf5_srv.response.cloud_data;
        original_cloud_list.push_back(cloud_data);
        if (index >= hdf5_srv.response.data_size - 1) {
            break;
        }
        index++;
    }
    for (int all_index = 0; all_index < original_cloud_list.size(); all_index++) {
        common_msgs::CloudData noize_cloud, final_cloud;
        final_cloud = original_cloud_list[all_index];
        auto origin_1 = UtilMsgData::extract_ins_cloudmsg(final_cloud, 1);
        auto centroid = NoizeCloudTransform::get_centroid(origin_1);
        NoizeCloudMake noize_object;
        if (util_.probability() < 0.6) {
            noize_cloud = UtilMsgData::concat_cloudmsg(noize_cloud, noize_object.noize_tube_small());
        }
        if (util_.probability() < 0.6) {
            noize_cloud = UtilMsgData::concat_cloudmsg(noize_cloud, noize_object.noize_tube_big());
        }
        centroid = NoizeCloudTransform::change_frame_id(centroid, sensor_frame_, world_frame_);
        noize_cloud = NoizeCloudTransform::translation_noize(noize_cloud, centroid);
        auto noize_cloud_final = NoizeCloudTransform::change_frame_id(noize_cloud, world_frame_, sensor_frame_);
        if (noize_cloud_final.x.size() > 0) {
            final_cloud = UtilMsgData::concat_cloudmsg(final_cloud, noize_cloud_final);
        }
        common_srvs::Hdf5RecordSegmentation record_srv;
        record_srv.request.the_number_of_dataset = original_cloud_list.size();
        record_srv.request.record_file_path = hdf5_record_file_path_;
        record_srv.request.cloud_data = final_cloud;
        Util::client_request(hdf5_record_client_, record_srv, hdf5_record_service_name_);
    }
    
}