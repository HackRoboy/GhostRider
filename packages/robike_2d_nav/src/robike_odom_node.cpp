#include "robike_2d_nav/robike_odom_node.h"

RobikeOdomNode::RobikeOdomNode(ros::NodeHandle nh, ros::NodeHandle pnh)
{
}

RobikeOdomNode::~RobikeOdomNode()
{

}

bool RobikeOdomNode::init()
{

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robike_odom_node");
    RobikeOdomNode odom_node(ros::NodeHandle(), ros::NodeHandle("~"));
    if (!odom_node.init())
        return false;
    else
        ros::spin();

    return 0;
}
