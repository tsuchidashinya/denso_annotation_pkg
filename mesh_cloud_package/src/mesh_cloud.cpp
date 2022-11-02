#pragma once
#include <mesh_cloud_package/mesh_cloud.hpp>>

using mesh_sampler::uniform_sampling;

/*
1: Nodehandle
2: object_name
3: mesh_topic_name
4: the number of mesh object
*/
MeshCloud::MeshCloud(ros::NodeHandle &nh)
    : nh_(nh),
      pnh_("~")
{
    hyper_parameter();
    server_ = nh_.advertiseService(mesh_service_name_, &MeshCloud::service_callback, this);
    visual_server_ = nh_.advertiseService("visual_mesh_service", &MeshCloud::visual_service_callback, this);

    print_parameter("sample_point is" + to_string(sample_points));
}

bool MeshCloud::visual_service_callback(denso_srvs::visual_serviceRequest &request, denso_srvs::visual_serviceResponse &response)
{
    visualize_data(request.the_number_of_mesh);
    response.owari = 1;
    return true;
}

void MeshCloud::initialize(denso_srvs::mesh_provide_serviceRequest request)
{
    mesh_pcl_clusters_.resize(request.the_number_of_mesh);
    mesh_cluster_pub_.resize(request.the_number_of_mesh);
    mesh_ros_.resize(request.the_number_of_mesh);
    for (int i = 0; i < request.the_number_of_mesh; i++)
    {
        mesh_cluster_pub_[i] = nh_.advertise<sensor_msgs::PointCloud2>(mesh_topic_name_ + "_" + to_string(i), 1);
    }
}

bool MeshCloud::service_callback(denso_srvs::mesh_provide_serviceRequest &request, denso_srvs::mesh_provide_serviceResponse &response)
{
    initialize(request);
    mesh_out_type out_data;
    // ros_print_parameter("tsuchida");
    out_data = make_mesh(request);
    ros_print_parameter("make_mesh");
    visualize_data(request.the_number_of_mesh);
    ros_print_parameter("visualize");
    ros_print_parameter("convert");
    response.mesh_data = out_data.mesh_data;
    response.pose_data = out_data.pose_data;
    return true;
}

mesh_out_type MeshCloud::make_mesh(denso_srvs::mesh_provide_serviceRequest request)
{
    print_parameter_and_name(request.the_number_of_mesh, "the number of mesh");
    for (int i = 0; i < request.tf_object_frame.size(); i++)
    {
        print_parameter(request.tf_object_frame[i]);
    }
    ros_print_parameter("tf_object finish");
    hyper_parameter_realtime();
    mesh_out_type out_data;
    out_data.mesh_data.resize(request.the_number_of_mesh);
    out_data.pose_data.resize(request.the_number_of_mesh);
    for (int i = 0; i < request.tf_object_frame.size(); i++)
    {
        PolygonMesh mesh;
        io::loadPolygonFileSTL(mesh_path_, mesh);
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        io::mesh2vtk(mesh, polydata);
        uniform_sampling(polydata, sample_points, mesh_pcl_clusters_[i]);
        tf::StampedTransform tf_2, tf_1, tf_gt;
        tf_1 = convert_stamped_trans(get_tf(request.tf_object_frame[i], world_frame_));
        pcl_ros::transformPointCloud(mesh_pcl_clusters_[i], mesh_pcl_clusters_[i], tf_1);
        tf_2 = convert_stamped_trans(get_tf(world_frame_, sensor_frame_));
        pcl_ros::transformPointCloud(mesh_pcl_clusters_[i], mesh_pcl_clusters_[i], tf_2);
        tf_gt = convert_stamped_trans(get_tf(request.tf_object_frame[i], sensor_frame_));
        out_data.pose_data[i] = stamped_to_pose(tf_gt);
        // mesh_pcl_clusters_[i] = downSample_VoxelGrid(mesh_pcl_clusters_[i], LEAF_SIZE_);
        out_data.mesh_data[i] = convert_pcl_to_segmsg(mesh_pcl_clusters_[i], background_instance_);
    }
    return out_data;
}

denso_msgs::pose_data MeshCloud::stamped_to_pose(tf::StampedTransform tf_stamped)
{
    denso_msgs::pose_data out_data;

    out_data.translation.x = tf_stamped.getOrigin().x();
    out_data.translation.y = tf_stamped.getOrigin().y();
    out_data.translation.z = tf_stamped.getOrigin().z();
    out_data.rotation.x = tf_stamped.getRotation().x();
    out_data.rotation.y = tf_stamped.getRotation().y();
    out_data.rotation.z = tf_stamped.getRotation().z();
    out_data.rotation.w = tf_stamped.getRotation().w();
    return out_data;
}

void MeshCloud::visualize_data(int the_number_of_mesh)
{
    for (int i = 0; i < the_number_of_mesh; i++)
    {
        sensor_msgs::PointCloud2 ros_pcl;
        ros_pcl = convert_pcl_to_ros(mesh_pcl_clusters_[i]);
        ros_pcl.header.frame_id = sensor_frame_;
        mesh_cluster_pub_[i].publish(ros_pcl);
    }
}

void MeshCloud::hyper_parameter()
{
    pnh_.getParam("sample_points", sample_points);
    pnh_.getParam("mesh_file_path", mesh_path_);
    pnh_.getParam("mesh_topic_name", mesh_topic_name_);
    pnh_.getParam("mesh_service_name", mesh_service_name_);
    pnh_.getParam("object_name", object_name_);
}

void MeshCloud::hyper_parameter_realtime()
{
    nh_.getParam("world_frame", world_frame_);
    nh_.getParam("backgound_instance", background_instance_);
    nh_.getParam("LEAF_SIZE", LEAF_SIZE_);
    nh_.getParam("sensor_frame", sensor_frame_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mesh_service");
    ros::NodeHandle nh;
    MeshCloud mesh(nh);
    ros::spin();
    return 0;
}