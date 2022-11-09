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
    timer_ = nh_.createTimer(ros::Duration(0.3), &MeshCloudServer::visualize_callback, this);
}

void MeshCloudServer::visualize_callback(const ros::TimerEvent &event)
{
    UtilSensor util_sensor;
    for (int i = 0; i < mesh_pcl_clusters_.size(); i++) {
        sensor_msgs::PointCloud2 pc = util_sensor.pcl_to_pc2(mesh_pcl_clusters_[i]);
        pc.header.frame_id = sensor_frame_;
        mesh_cluster_pub_[i].publish(pc);
    }
}

void MeshCloudServer::initialize(anno_srvs::MeshCloudServiceRequest request)
{
    int size = request.multi_object_info.size();
    mesh_pcl_clusters_.resize(size);
    mesh_cluster_pub_.resize(size);
    for (int i = 0; i < size; i++)
    {
        mesh_cluster_pub_[i] = nh_.advertise<sensor_msgs::PointCloud2>(request.multi_object_info[i].object_name + "_" + std::to_string(i), 1);
    }
}

bool MeshCloudServer::service_callback(anno_srvs::MeshCloudServiceRequest &request, anno_srvs::MeshCloudServiceResponse &response)
{
    initialize(request);
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
    for (int i = 0; i < tf_name_size; i++)
    {
        pcl::PolygonMesh mesh;
        std::string mesh_path = ros::package::getPath("mesh_cloud_package");
        mesh_path = UtilBase::join(mesh_path, "mesh");
        mesh_path = UtilBase::join(mesh_path, request.multi_object_info[i].object_name + ".stl");
        pcl::io::loadPolygonFileSTL(mesh_path, mesh);
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        pcl::io::mesh2vtk(mesh, polydata);
        uniform_sampling(polydata, sample_points, mesh_pcl_clusters_[i]);
        tf::StampedTransform sensor_to_world, world_to_object, sensor_to_object;
        
        world_to_object = UtilBase::make_stamped_trans(tf_basic_.get_tf(request.multi_object_info[i].tf_name, world_frame_));
        pcl_ros::transformPointCloud(mesh_pcl_clusters_[i], mesh_pcl_clusters_[i], world_to_object);
        sensor_to_world = UtilBase::make_stamped_trans(tf_basic_.get_tf(world_frame_, sensor_frame_));
        UtilBase::message_show(world_frame_, sensor_frame_);
        TfBasic::tf_data_show(tf_basic_.get_tf(world_frame_, sensor_frame_), "tf_sensor_name");
        pcl_ros::transformPointCloud(mesh_pcl_clusters_[i], mesh_pcl_clusters_[i], sensor_to_world);
        sensor_to_object = UtilBase::make_stamped_trans(tf_basic_.get_tf(request.multi_object_info[i].tf_name, sensor_frame_));
        out_data.pose_data[i] = stamped_to_pose(sensor_to_object);
        out_data.mesh_data[i] = UtilSensor::pcl_to_cloudmsg(mesh_pcl_clusters_[i]);
        out_data.mesh_data[i].cloud_name = request.multi_object_info[i].tf_name;
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

void MeshCloudServer::visualize_data(int the_number_of_mesh)
{
    for (int i = 0; i < the_number_of_mesh; i++)
    {
        sensor_msgs::PointCloud2 ros_pcl;
        ros_pcl = UtilSensor::pcl_to_pc2(mesh_pcl_clusters_[i]);
        ros_pcl.header.frame_id = sensor_frame_;
        mesh_cluster_pub_[i].publish(ros_pcl);
    }
}

void MeshCloudServer::set_parameter()
{
    pnh_.getParam("mesh_cloud", param_list);
    sample_points = param_list["sample_points"];
    mesh_service_name_ = static_cast<std::string>(param_list["mesh_service_name"]);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    LEAF_SIZE_ = param_list["LEAF_SIZE"];
}
