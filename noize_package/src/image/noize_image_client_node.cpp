#include <noize_package/image/noize_image_client.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hdf5_open_data_process");
    ros::NodeHandle nh;
    Util util;
    NoizeImageClient noize_image_client(nh);
    for (int i = 0; i < 100; i++) {
        if (util.probability() < 1) {
            noize_image_client.noize_image_main();
        }
        else {
            noize_image_client.restore_image();
        }
        ros::WallDuration(2.0).sleep();
    }
    return 0;
}