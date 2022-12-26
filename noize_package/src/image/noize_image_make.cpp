#include <noize_package/image/noize_image_make.hpp>

NoizeImageMake::NoizeImageMake() :
pnh_("~")
{
    set_parameter();
}

void NoizeImageMake::set_parameter()
{
    pnh_.getParam("noize_image_make", param_list);
    image_height_ = static_cast<int>(param_list["image_height"]);
    image_width_ = static_cast<int>(param_list["image_width"]);
    divide_threshold_ = static_cast<int>(param_list["divide_threshold"]);
}

cv::Mat NoizeImageMake::plane_image(int r, int g, int b)
{
    cv::Mat img(cv::Size(image_width_, image_height_), CV_8UC3, cv::Scalar(b, g, r));
    return img;
}



cv::Mat NoizeImageMake::circle(cv::Mat image, cv::Point point, int radious, cv::Scalar scalar)
{
    cv::circle(image, point, radious, scalar, -1);
    return image;
}

cv::Mat NoizeImageMake::circle_line(cv::Mat image, cv::Point point, int radious, cv::Scalar scalar, int thickness)
{
    cv::circle(image, point, radious, scalar, thickness);
    return image;
}



cv::Mat NoizeImageMake::linear_line(cv::Mat image, cv::Point point1, cv::Point point2, cv::Scalar scalar, int thickness)
{
    cv::line(image, point1, point2, scalar, thickness);
    return image;
}

cv::Mat NoizeImageMake::rectangle_line(cv::Mat image, cv::Point point1, cv::Point point2, cv::Scalar scalar, int thickness)
{
    cv::rectangle(image, point1, point2, scalar, thickness);
    return image;
}

cv::Mat NoizeImageMake::rectangle(cv::Mat image, cv::Point point1, cv::Point point2, cv::Scalar scalar)
{
    cv::rectangle(image, point1, point2, scalar, -1);
    return image;
}

cv::Mat NoizeImageMake::ellipse_line(cv::Mat image, cv::Point point1, cv::Point point2, int tilt_angle, int end_angle, cv::Scalar scalar, int thickness)
{
    cv::ellipse(image, point1, point2, tilt_angle, 0, end_angle, scalar, thickness);
    return image;
}

cv::Mat NoizeImageMake::ellipse(cv::Mat image, cv::Point point1, cv::Point point2, int tilt_angle, int end_angle, cv::Scalar scalar)
{
    cv::ellipse(image, point1, point2, tilt_angle, 0, end_angle, scalar, -1);
    return image;
}

void NoizeImageMake::randomize_parameter_small()
{
    x_ = util_.random_int(0, image_width_-1);
    y_ = util_.random_int(0, image_height_-1);
    std::vector<int> swap_list_x, swap_list_y;
    while (ros::ok()) {
        swap_list_x = Util::get_swap_list(util_.random_int(0, image_width_-1), util_.random_int(0, image_width_-1));
        int subtract = swap_list_x[1] - swap_list_x[0];
        if (subtract > 3 && subtract < divide_threshold_) 
            break;
    } 
    while (ros::ok()) {
        swap_list_y = Util::get_swap_list(util_.random_int(0, image_height_-1), util_.random_int(0, image_height_-1));
        int subtract = swap_list_y[1] - swap_list_y[0];
        if (subtract > 3 && subtract < divide_threshold_) 
            break;
        
    } 
    x1_ = swap_list_x[0];
    x2_ = swap_list_x[1];
    y1_ = swap_list_y[0];
    y2_ = swap_list_y[1];
    blue_ = util_.random_int(0, 255);
    green_ = util_.random_int(0, 255);
    red_ = util_.random_int(0, 255);
    auto swap_list = Util::get_swap_list(image_width_, image_height_);
    radious_ = util_.random_int(1, divide_threshold_);
    thick_circle_ = util_.random_int(1, radious_);
    auto swap_thick = Util::get_swap_list(y2_ - y1_, x2_ - x1_);
    thick_other_ = util_.random_int(1, swap_thick[0]);
    tilt_angle_ = util_.random_int(0, 360);
    if (util_.probability() < 0.5) {
        end_angle_ = 360;
    }
    else {
        end_angle_ = util_.random_int(0, 360);
    }
}

void NoizeImageMake::randomize_parameter_big()
{
    x_ = util_.random_int(0, image_width_-1);
    y_ = util_.random_int(0, image_height_-1);
    std::vector<int> swap_list_x, swap_list_y;
    while (ros::ok()) {
        swap_list_x = Util::get_swap_list(util_.random_int(0, image_width_-1), util_.random_int(0, image_width_-1));
        int subtract = swap_list_x[1] - swap_list_x[0];
        if (subtract > divide_threshold_) 
            break;
    } 
    while (ros::ok()) {
        swap_list_y = Util::get_swap_list(util_.random_int(0, image_height_-1), util_.random_int(0, image_height_-1));
        int subtract = swap_list_y[1] - swap_list_y[0];
        if (subtract > divide_threshold_) 
            break;
    } 
    x1_ = swap_list_x[0];
    x2_ = swap_list_x[1];
    y1_ = swap_list_y[0];
    y2_ = swap_list_y[1];
    blue_ = util_.random_int(0, 255);
    green_ = util_.random_int(0, 255);
    red_ = util_.random_int(0, 255);
    auto swap_list = Util::get_swap_list(image_width_, image_height_);
    radious_ = util_.random_int(divide_threshold_, swap_list[0] / 3);
    thick_circle_ = util_.random_int(1, radious_);
    auto swap_thick = Util::get_swap_list(y2_ - y1_, x2_ - x1_);
    thick_other_ = util_.random_int(1, swap_thick[0]);
    tilt_angle_ = util_.random_int(0, 360);
    if (util_.probability() < 0.5) {
        end_angle_ = 360;
    }
    else {
        end_angle_ = util_.random_int(0, 360);
    }
}
cv::Mat NoizeImageMake::plane_image_random()
{
    randomize_parameter_small();
    return plane_image(red_, green_, blue_);
}

cv::Mat NoizeImageMake::circle_random(cv::Mat image, SizeOption size)
{
    if (size == SizeOption::small) {
        randomize_parameter_small();
    }
    else {
        randomize_parameter_big();
    }
    if (util_.random_float(0, 1) > 0.5) {
        return circle(image, cv::Point(x_, y_), radious_, cv::Scalar(blue_, green_, red_));
    }
    else {
        return circle_line(image, cv::Point(x_, y_), radious_, cv::Scalar(blue_, green_, red_), thick_circle_);
    }
}

cv::Mat NoizeImageMake::linear_line_random(cv::Mat image, SizeOption size)
{
    if (size == SizeOption::small) {
        randomize_parameter_small();
    }
    else {
        randomize_parameter_big();
    }
    return linear_line(image, cv::Point(x1_, y1_), cv::Point(x2_, y2_), cv::Scalar(blue_, green_, red_), thick_other_);
}

cv::Mat NoizeImageMake::rectangle_random(cv::Mat image, SizeOption size)
{
    if (size == SizeOption::small) {
        randomize_parameter_small();
    }
    else {
        randomize_parameter_big();
    }
    if (util_.random_float(0, 1) > 0.5) {
        return rectangle(image, cv::Point(x1_, y1_), cv::Point(x2_, y2_), cv::Scalar(blue_, green_, red_));
    }
    else {
        return rectangle_line(image, cv::Point(x1_, y1_), cv::Point(x2_, y2_), cv::Scalar(blue_, green_, red_), thick_other_);
    }
}

cv::Mat NoizeImageMake::ellipse_random(cv::Mat image, SizeOption size)
{
    if (size == SizeOption::small) {
        randomize_parameter_small();
    }
    else {
        randomize_parameter_big();
    }
    if (util_.random_float(0, 1) > 0.5) {
        return ellipse(image, cv::Point(x_, y_), cv::Point(x2_, x1_), tilt_angle_, end_angle_, cv::Scalar(blue_, green_, red_));
    }
    else {
        return ellipse_line(image, cv::Point(x_, y_), cv::Point(x2_, x1_), tilt_angle_, end_angle_, cv::Scalar(blue_, green_, red_), thick_other_);
    }
}

