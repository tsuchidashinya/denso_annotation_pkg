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
    for (int i = 0; i < request.multi_object_info.size(); i++) {
        pcl::PointCloud<PclXyz> mesh_data;
        int index = Util::find_element_vector(mesh_name_list_, request.multi_object_info[i].object_name);
        if (index == -1) {
            mesh_data = create_mesh(request.multi_object_info[i].object_name);
            mesh_name_list_.push_back(request.multi_object_info[i].object_name);
            mesh_stack_list_.push_back(mesh_data);
        }
        else {
            mesh_data = mesh_stack_list_[index];
        }
        mesh_data = transform_mesh(mesh_data, request.multi_object_info[i].tf_name);
        common_msgs::CloudData mesh_cloud = UtilMsgData::pcl_to_cloudmsg(mesh_data);
        mesh_cloud.object_name = request.multi_object_info[i].object_name;
        mesh_cloud.tf_name = request.multi_object_info[i].tf_name;
        response.mesh.push_back(mesh_cloud);
        tf::StampedTransform sensor_to_object;
        sensor_to_object = Util::make_stamped_trans(tf_func_.tf_listen(request.multi_object_info[i].tf_name, sensor_frame_));
        response.pose.push_back(stamped_to_pose(sensor_to_object));
    }
    return true;
}

pcl::PointCloud<PclXyz> MeshCloudServer::create_mesh(std::string object_name)
{
    pcl::PointCloud<PclXyz> outdata;
    pcl::PolygonMesh mesh_polygon;
    std::string mesh_path = ros::package::getPath("mesh_cloud_package");
    mesh_path = Util::join(mesh_path, "mesh");
    mesh_path = Util::join(mesh_path, object_name + ".stl");
    pcl::io::loadPolygonFileSTL(mesh_path, mesh_polygon);
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::mesh2vtk(mesh_polygon, polydata);
    uniform_sampling(polydata, sample_points, outdata);
    return outdata;
}

pcl::PointCloud<PclXyz> MeshCloudServer::transform_mesh(pcl::PointCloud<PclXyz> mesh_data, std::string tf_name)
{
    tf::StampedTransform sensor_to_world, world_to_object;
    world_to_object = Util::make_stamped_trans(tf_func_.tf_listen(tf_name, world_frame_));
    pcl_ros::transformPointCloud(mesh_data, mesh_data, world_to_object);
    sensor_to_world = Util::make_stamped_trans(tf_func_.tf_listen(world_frame_, sensor_frame_));
    pcl_ros::transformPointCloud(mesh_data, mesh_data, sensor_to_world);
    return mesh_data;
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
