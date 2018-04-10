#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport imageTransport(nh);
    image_transport::Publisher publisher = imageTransport.advertise("camera/image", 1);

    cv::Mat image;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(5);
    while (nh.ok()) {

        image = cv::imread("/home/adept/Pictures/person_in_traffic.jpg", CV_LOAD_IMAGE_COLOR);

        if (!image.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            publisher.publish(msg);
            cv::waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
