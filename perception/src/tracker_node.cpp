#include <ros/ros.h>
#include <image_transport/image_transport.h>

void trackerCallback(const sensor_msgs::ImageConstPtr& image)
{
    // ROS_INFO("Message received!");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscriber = nodeHandle.subscribe("camera/image", 1, trackerCallback);

    ros::spin();
}
