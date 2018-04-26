#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

class WorldPositioner
{
    public:
        WorldPositioner(const ros::NodeHandle nodeHandle);

    private:
        ros::NodeHandle nodeHandle;

        image_transport::Subscriber depthSubscriber;
        image_transport::ImageTransport depthImageTransport;

        ros::Subscriber boundingBoxSubscriber;

        void depthCallback(const sensor_msgs::ImageConstPtr& msg);
        void boundingBoxCallback(const darknet_ros_msgs::BoundingBoxes& msg);
};
