#include <annotation_client_pkg/semantic_segmentation_client.hpp>

SemanticSegmentation::SemanticSegmentation(ros::NodeHandle &nh):
    nh_(nh),
    pnh_("~")
{
    set_paramenter();
}

void SemanticSegmentation::set_paramenter()
{
    pnh_.getParam("annotation_main", param_list);
    UtilBase::message_show("LeafSize: ", param_list["LEAF_SIZE"]);
}

void SemanticSegmentation::main()
{
    UtilBase::message_show("", "start!");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_segmentation");
    ros::NodeHandle nh;
    SemanticSegmentation semseg(nh);
    semseg.main();
    return 0;
}