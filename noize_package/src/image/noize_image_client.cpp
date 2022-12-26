#include <noize_package/image/noize_image_client.hpp>

NoizeImageClient::NoizeImageClient(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_parameter();
    vis_image_client_ = nh_.serviceClient<common_srvs::VisualizeImage>(vis_image_service_name_);
}

void NoizeImageClient::set_parameter()
{
    pnh_.getParam("noize_image_client", param_list);
    vis_image_service_name_ = static_cast<std::string>(param_list["visualize_image_service_name"]);
    small_max_count_ = static_cast<int>(param_list["small_max_count"]);
    big_max_count_ = static_cast<int>(param_list["big_max_count"]);
}

void NoizeImageClient::restore_image()
{
    NoizeImageMake noize_image;
    float scale = 0.7;
    auto img = noize_image.plane_image(255 * scale, 255 * scale, 255 * scale);
    auto sensor_image = UtilMsgData::cvimg_to_rosimg(img, sensor_msgs::image_encodings::BGR8);
    common_srvs::VisualizeImage vis_img_srv;
    vis_img_srv.request.image_list.push_back(sensor_image);
    vis_img_srv.request.topic_name_list.push_back("display_object_topic");
    Util::client_request(vis_image_client_, vis_img_srv, vis_image_service_name_);
}

void NoizeImageClient::noize_image_main()
{
    NoizeImageMake noize_image;
    auto img = noize_image.plane_image_random();
    for (int i = 0; i < util_.random_int(1, big_max_count_); i++) {
        auto probably = util_.probability();
        if (probably < 0.25) {
            img = noize_image.circle_random(img, SizeOption::big);
        }
        else if (probably < 0.6) {
            img = noize_image.linear_line_random(img, SizeOption::big);
        }
        else if (probably < 0.8) {
            img = noize_image.rectangle_random(img, SizeOption::big);
        }
        else if (probably < 1) {
            img = noize_image.ellipse_random(img, SizeOption::big);
        }
        
    }
    for (int i = 0; i < util_.random_int(1, small_max_count_); i++) {
        auto probably = util_.probability();
        if (probably < 0.25) {
            img = noize_image.circle_random(img, SizeOption::small);
        }
        else if (probably < 0.6) {
            img = noize_image.linear_line_random(img, SizeOption::small);
        }
        else if (probably < 0.8) {
            img = noize_image.rectangle_random(img, SizeOption::small);
        }
        else if (probably < 1) {
            img = noize_image.ellipse_random(img, SizeOption::small);
        }
        
    }
    auto sensor_image = UtilMsgData::cvimg_to_rosimg(img, sensor_msgs::image_encodings::BGR8);
    common_srvs::VisualizeImage vis_img_srv;
    vis_img_srv.request.image_list.push_back(sensor_image);
    vis_img_srv.request.topic_name_list.push_back("display_object_topic");
    Util::client_request(vis_image_client_, vis_img_srv, vis_image_service_name_);
}