#include <annotation_common/hdf5_open_data_process.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hdf5_open_data_process");
    ros::NodeHandle nh;
    Hdf5OpenDataProcess hdf5_open_process(nh);
    hdf5_open_process.main();
    return 0;
}