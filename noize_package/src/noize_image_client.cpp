#include <noize_package/noize_client.hpp>

NoizeClient::NoizeClient(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_parameter();
    vis_image_client_ = nh_.serviceClient<common_srvs::VisualizeImage>(vis_image_service_name_);
}

void NoizeClient::set_parameter()
{
    pnh_.getParam("noize_image_client", param_list);
    vis_image_service_name_ = static_cast<std::string>(param_list["visualize_image_service_name"]);
}

void NoizeClient::main()
{
    NoizeImageMake noize_image;
    auto img = noize_image.plane_image(255, 0, 0);
    auto sensor_image = UtilMsgData::cvimg_to_rosimg(img, sensor_msgs::image_encodings::BGR8);
    common_srvs::VisualizeImage vis_img_srv;
    vis_img_srv.request.image_list.push_back(sensor_image);
    vis_img_srv.request.topic_name_list.push_back("display_object_topic");
    Util::client_request(vis_image_client_, vis_img_srv, vis_image_service_name_);
}