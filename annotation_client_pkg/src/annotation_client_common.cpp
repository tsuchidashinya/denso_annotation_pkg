#include <annotation_client_pkg/annotation_client.hpp>

AnnotationClient::AnnotationClient(ros::NodeHandle &nh):
    nh_(nh),
    pnh_("~"),
    counter_(0),
    decide_gazebo_object_()
{
    set_paramenter();
    

    if (object_list_[0] == "HV8" || object_list_[0] == "HV6") {
        decide_gazebo_object_.set_decice_pose_option(DecidePoseOption::Head);
    }
    else {
        decide_gazebo_object_.set_decice_pose_option(DecidePoseOption::FullRandom);
    }
    
}

common_msgs::CloudData AnnotationClient::crop_cloudmsg(common_msgs::CloudData input_cloud) {
    pcl::PointCloud<pcl::PointXYZL> pcl_data = UtilMsgData::cloudmsg_to_pclLabel(input_cloud);
    cloud_process_.set_crop_frame(sensor_frame_, world_frame_);
    pcl_data = cloud_process_.cropbox_segmenter(pcl_data);
    return UtilMsgData::pclLabel_to_cloudmsg(pcl_data);
};

common_srvs::MeshCloudService AnnotationClient::mesh_request(std::string tf_name)
{
    common_msgs::ObjectInfo mesh_input;
    common_srvs::MeshCloudService mesh_srv;
    mesh_input.object_name = object_list_[0];
    mesh_input.tf_name = tf_name;
    mesh_srv.request.multi_object_info.clear();
    mesh_srv.request.multi_object_info.push_back(mesh_input);
    Util::client_request(mesh_client_, mesh_srv, mesh_service_name_);
    return mesh_srv;
}

common_msgs::CloudData AnnotationClient::nearest_extractor(common_msgs::CloudData cloud, common_srvs::MeshCloudService mesh_srv, int instance)
{
    return SpaceHandlingLibrary::search_nearest_point(cloud, mesh_srv.response.mesh[0], instance, nearest_radious_);
}

void AnnotationClient::tf_broadcast_request(geometry_msgs::TransformStamped trans_stamp)
{
    common_srvs::TfBroadcastService tf_srv;
    tf_srv.request.broadcast_tf = trans_stamp;
    tf_srv.request.tf_name = trans_stamp.child_frame_id;
    Util::client_request(tf_br_client_, tf_srv, tf_br_service_name_);
}

void AnnotationClient::visualize_request(common_msgs::CloudData cloud, std::string topic_name)
{
    common_srvs::VisualizeCloud visual_srv;
    visual_srv.request.cloud_data_list.clear();
    visual_srv.request.topic_name_list.clear();
    visual_srv.request.cloud_data_list.push_back(cloud);
    visual_srv.request.topic_name_list.push_back(topic_name);
    Util::client_request(visualize_client_, visual_srv, visualize_service_name_);
}