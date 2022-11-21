#include <annotation_client_pkg/annotation_client.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_segmentation");
    ros::NodeHandle nh;
    AnnotationClient annotation_main(nh);
    for (int i = 0; i < annotation_main.the_number_of_dataset_; i++) {
        annotation_main.main();
        // ros::Duration(0.1).sleep();
    }
    return 0;
}