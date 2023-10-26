/**
 * @file tracker.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief Implementation of the Test class for depth image processing.
 * @version 0.1
 * @date 2021-04-08
 *
 * @copyright Copyright (c) 2021
 *
 */

/* INCLUDE FILES */
#include "detector/tracker.h"
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

/* CLASS METHODS DEFINITION */

/**
 * @brief Subscription callback for depth images.
 *
 * This function gets triggered when a new depth image message is received.
 * It processes the depth image and displays it using OpenCV.
 *
 * @param msg Pointer to the received depth image message.
 */
void Test::test(const sensor_msgs::ImageConstPtr &msg) {
    // Conversion from ROS image message to OpenCV image
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);

    // Loop through the image pixels and log the depth values
    for(int i = 0; i < cv_ptr->image.rows; i++) {
        for(int j = 0; j < cv_ptr->image.cols; j++) {
            float value = cv_ptr->image.at<float>(i, j);
            ROS_INFO("Depth value at (%d, %d): %f", i, j, value);
        }
    }

    // Display the received image using OpenCV
    cv::namedWindow("Received Image", cv::WINDOW_NORMAL);
    cv::imshow("Received Image", cv_ptr->image);
    cv::waitKey(30);
}

/**
 * @brief Initialization function for the Test class.
 *
 * This function initializes necessary subscribers and retrieves parameters from the ROS parameter server.
 */
Test::Test(ros::NodeHandle nh) {
   
    ReadParameters(nh_);
    
     nh.getParam("/detector_node/depth_topic", k_depth_topic_);
      image_transport::ImageTransport it_depth_rst(nh_);
    depth_sub_ = nh_.subscribe(k_depth_topic_, 1, &Test::test, this);
   
}

// MAIN FUNCTION
int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "test_node");

    // Create a NodeHandle
    ros::NodeHandle nh;

    // Create an instance of the Test class
    Test testInstance(nh);

    // Keep the node running
    ros::spin();

    return 0;
}
