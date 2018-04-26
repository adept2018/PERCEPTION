#include "WorldPositioner.h"
#include <iostream>

WorldPositioner::WorldPositioner(const ros::NodeHandle nh)
    : nodeHandle(nh),
      depthImageTransport(nh)
{
    ROS_INFO("Running WorldPositioner constructor!");

    depthSubscriber = depthImageTransport.subscribe("depth/depth_registered",
                                                    1,
                                                    &WorldPositioner::depthCallback,
                                                    this);

    boundingBoxSubscriber = nodeHandle.subscribe("darknet_ros/bounding_boxes",
                                                 1,
                                                 &WorldPositioner::boundingBoxCallback,
                                                 this);
}

void WorldPositioner::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Running depth callback!");
}

void WorldPositioner::boundingBoxCallback(const darknet_ros_msgs::BoundingBoxes& msg)
{
    ROS_INFO("Running bounding box callback!");
}
