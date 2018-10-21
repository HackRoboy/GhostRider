#include <ros/ros.h>

class RobikeOdomNode {
public:
    RobikeOdomNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~RobikeOdomNode();

    bool init();
};
