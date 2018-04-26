#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "WorldPositioner.h"

void trackerCallback(const sensor_msgs::ImageConstPtr& image)
{
    ROS_INFO("Depth image received!");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nodeHandle;

    WorldPositioner worldPositioner(nodeHandle);

    ros::spin();
}
