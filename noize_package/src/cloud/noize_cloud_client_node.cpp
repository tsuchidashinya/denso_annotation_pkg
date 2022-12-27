#include <noize_package/cloud/noize_cloud_client.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hdf5_open_data_process");
    ros::NodeHandle nh;
    NoizeCloudClient noize_cloud_client(nh);
    for (int i = 0; i < 100; i++) {
        noize_cloud_client.main();
        ros::WallDuration(2.0).sleep();
    }
    return 0;
}