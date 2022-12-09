#include <annotation_common/hdf5_open_data_process.hpp>

Hdf5OpenDataProcess::Hdf5OpenDataProcess(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_parameter();
    hdf5_open_client_ = nh_.serviceClient<common_srvs::Hdf5OpenSegmentationService>(hdf5_open_service_name_);
    visualize_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(visualize_service_name_);
}

void Hdf5OpenDataProcess::set_parameter()
{
    pnh_.getParam("noize_make_client", param_list);
    hdf5_open_service_name_ = static_cast<std::string>(param_list["hdf5_open_service_name"]);
    visualize_service_name_ = static_cast<std::string>(param_list["visualize_service_name"]);
}

void Hdf5OpenDataProcess::main()
{
    int data_size;
    nh_.getParam("hdf5_data_size", data_size);
    for (int i = 1; i <= data_size; i++) {
        common_srvs::Hdf5OpenSegmentationService hdf5_open_srv;
        hdf5_open_srv.request.index = i;
        Util::client_request(hdf5_open_client_, hdf5_open_srv, hdf5_open_service_name_);
        common_srvs::VisualizeCloud visual_srv;
        visual_srv.request.cloud_data_list.push_back(hdf5_open_srv.response.cloud_data);
        visual_srv.request.topic_name_list.push_back("index_" + std::to_string(i));
        Util::client_request(visualize_client_, visual_srv, visualize_service_name_);
    }
}