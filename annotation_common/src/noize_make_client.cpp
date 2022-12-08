#include <annotation_common/hdf5_open_data_process.hpp>

Hdf5OpenDataProcess::Hdf5OpenDataProcess(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_parameter();
    hdf5_open_client_ = nh_.serviceClient<common_srvs::Hdf5OpenService>(hdf5_open_service_name_);
}

void Hdf5OpenDataProcess::set_parameter()
{
    pnh_.getParam("noize_make_client", param_list);

}

void Hdf5OpenDataProcess::main()
{
    
}