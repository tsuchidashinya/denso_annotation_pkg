#include <annotation_client_pkg/annotation_client.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_segmentation");
    ros::NodeHandle nh;
    AnnotationClient annotation_main(nh);
    int counter;
    for (int i = 0; i < annotation_main.the_number_of_dataset_; i++) {
        
        if (annotation_main.main()) {
            break;
        };
        // ros::Duration(0.1).sleep();
        Util::message_show("Progress rate", std::to_string(i + 1) + "/" + std::to_string(annotation_main.the_number_of_dataset_));
        nh.getParam("record_counter", counter);
        if (counter >= annotation_main.the_number_of_dataset_) {
            break;
        }
    }
    return 0;
}