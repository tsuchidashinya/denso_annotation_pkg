#include <mesh_cloud_package/mesh_cloud.hpp>

/*
1: Nodehandle
2: object_name
3: mesh_topic_name
4: the number of mesh object
*/
MeshCloudServer::MeshCloudServer(ros::NodeHandle &nh)
    : nh_(nh),
      pnh_("~")
{
    set_parameter();
    server_ = nh_.advertiseService(mesh_service_name_, &MeshCloudServer::service_callback, this);
}

bool MeshCloudServer::service_callback(anno_srvs::MeshCloudServiceRequest &request, anno_srvs::MeshCloudServiceResponse &response)
{
    MeshOutType out_data;
    out_data = make_mesh(request);
    response.mesh = out_data.mesh_data;
    response.pose = out_data.pose_data;
    return true;
}

MeshOutType MeshCloudServer::make_mesh(anno_srvs::MeshCloudServiceRequest request)
{
    int tf_name_size = request.multi_object_info.size();
    MeshOutType out_data;
    out_data.mesh_data.resize(tf_name_size);
    out_data.pose_data.resize(tf_name_size);
    std::vector<pcl::PointCloud<PclXyz>> mesh_pcl_list;
    mesh_pcl_list.resize(tf_name_size);
    for (int i = 0; i < tf_name_size; i++)
    {
        pcl::PolygonMesh mesh;
        std::string mesh_path = ros::package::getPath("mesh_cloud_package");
        mesh_path = Util::join(mesh_path, "mesh");
        mesh_path = Util::join(mesh_path, request.multi_object_info[i].object_name + ".stl");
        // Util::message_show("load_poligon_before", "ok");
        pcl::io::loadPolygonFileSTL(mesh_path, mesh);
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        // Util::message_show("mesh2vtk", "ok");
        pcl::io::mesh2vtk(mesh, polydata);
        // Util::message_show("uniform_sampling", "ok");
        uniform_sampling(polydata, sample_points, mesh_pcl_list[i]);
        // Util::message_show("mesh_pcl_size", mesh_pcl_list[i].points.size());
        tf::StampedTransform sensor_to_world, world_to_object, sensor_to_object;
        // Util::message_show("make_stamped_trans", "ok");
        world_to_object = Util::make_stamped_trans(tf_func_.tf_listen(request.multi_object_info[i].tf_name, world_frame_));
        // Util::message_show("tf_multi_name", request.multi_object_info[i].tf_name);
        // Util::message_show("pcl_ros_1", "ok");
        pcl_ros::transformPointCloud(mesh_pcl_list[i], mesh_pcl_list[i], world_to_object);
        // Util::message_show("pcl_ros_2", "ok");
        sensor_to_world = Util::make_stamped_trans(tf_func_.tf_listen(world_frame_, sensor_frame_));
        // TfFunction::tf_data_show(tf_func_.tf_listen(world_frame_, sensor_frame_), "tf_sensor_name");
        pcl_ros::transformPointCloud(mesh_pcl_list[i], mesh_pcl_list[i], sensor_to_world);
        // Util::message_show("make_stamped_trams", "ok");
        sensor_to_object = Util::make_stamped_trans(tf_func_.tf_listen(request.multi_object_info[i].tf_name, sensor_frame_));
        out_data.pose_data[i] = stamped_to_pose(sensor_to_object);
        out_data.mesh_data[i] = UtilMsgData::pcl_to_cloudmsg(mesh_pcl_list[i]);
        out_data.mesh_data[i].tf_name = request.multi_object_info[i].tf_name;
        out_data.mesh_data[i].object_name = request.multi_object_info[i].object_name;
    }   
    return out_data;
}

common_msgs::PoseData MeshCloudServer::stamped_to_pose(tf::StampedTransform tf_stamped)
{
    common_msgs::PoseData out_data;
    out_data.trans.x = tf_stamped.getOrigin().x();
    out_data.trans.y = tf_stamped.getOrigin().y();
    out_data.trans.z = tf_stamped.getOrigin().z();
    out_data.rot.x = tf_stamped.getRotation().x();
    out_data.rot.y = tf_stamped.getRotation().y();
    out_data.rot.z = tf_stamped.getRotation().z();
    out_data.rot.w = tf_stamped.getRotation().w();
    return out_data;
}


void MeshCloudServer::set_parameter()
{
    pnh_.getParam("mesh_cloud", param_list);
    sample_points = param_list["sample_points"];
    mesh_service_name_ = static_cast<std::string>(param_list["mesh_service_name"]);
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    LEAF_SIZE_ = param_list["LEAF_SIZE"];
}
