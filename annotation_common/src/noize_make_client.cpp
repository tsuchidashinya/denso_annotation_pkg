#include <annotation_common/hdf5_open_data_process.hpp>

Hdf5OpenDataProcess::Hdf5OpenDataProcess(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_parameter();
    
}