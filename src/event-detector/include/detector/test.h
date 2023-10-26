// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

class Test {
public:
    // Constructor
    Test(ros::NodeHandle nh);

    // Member function to handle depth image callback
    void test(const sensor_msgs::ImageConstPtr &msg);

private:
    // NodeHandle for interacting with ROS
    ros::NodeHandle nh_;

    // Subscriber for depth images
    ros::Subscriber depth_sub_;

    // Topic name for depth images (this seems to be fetched from parameters, so making it a member variable)
    std::string k_depth_topic_;
};
