#include <noize_package/image/noize_image_make.hpp>

NoizeImageMake::NoizeImageMake() :
pnh_("~")
{
    ;
}

cv::Mat NoizeImageMake::plane_image(int r, int g, int b)
{
    cv::Mat img(cv::Size(2064, 1544), CV_8UC3, cv::Scalar(b, g, r));
    return img;
}