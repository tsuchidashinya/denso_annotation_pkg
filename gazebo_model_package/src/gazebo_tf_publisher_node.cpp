#include <gazebo_model_package/gazebo_tf_publisher.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_model_tf");
    ros::NodeHandle nh;
    GazeboTfPublisher model(nh);
    ros::spin();
}
