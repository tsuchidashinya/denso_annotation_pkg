#pragma once
#include <util_package/util.hpp>
#include <common_msgs/CloudData.h>
#include <sensor_msgs/Image.h>

enum SizeOption 
{
    small, 
    big
};

class NoizeImageMake
{
public:
    NoizeImageMake();
    void set_parameter();
    void randomize_parameter_small();
    void randomize_parameter_big();
    cv::Mat plane_image(int, int, int);
    cv::Mat plane_image_random();
    cv::Mat circle(cv::Mat, cv::Point, int, cv::Scalar);
    cv::Mat circle_line(cv::Mat, cv::Point, int, cv::Scalar, int);
    cv::Mat linear_line(cv::Mat, cv::Point, cv::Point, cv::Scalar, int);
    cv::Mat rectangle_line(cv::Mat, cv::Point, cv::Point, cv::Scalar, int);
    cv::Mat rectangle(cv::Mat, cv::Point, cv::Point, cv::Scalar);
    cv::Mat ellipse_line(cv::Mat, cv::Point, cv::Point, int, int, cv::Scalar, int);
    cv::Mat ellipse(cv::Mat, cv::Point, cv::Point, int, int, cv::Scalar);
    cv::Mat circle_random(cv::Mat, SizeOption);
    cv::Mat linear_line_random(cv::Mat, SizeOption);
    cv::Mat rectangle_random(cv::Mat, SizeOption);
    cv::Mat ellipse_random(cv::Mat, SizeOption);
     
private:
    ros::NodeHandle pnh_;
    XmlRpc::XmlRpcValue param_list;
    int image_width_, image_height_;
    int x_, y_, x1_, y1_, x2_, y2_;
    int red_, green_, blue_;
    int radious_;
    int thick_circle_, thick_other_;
    int tilt_angle_, end_angle_;
    int divide_threshold_;
    Util util_;
};