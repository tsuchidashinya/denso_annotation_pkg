#include <noize_package/noize_client.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hdf5_open_data_process");
    ros::NodeHandle nh;
    NoizeClient hdf5_open_process(nh);
    hdf5_open_process.main();
    return 0;
}