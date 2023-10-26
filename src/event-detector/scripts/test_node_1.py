#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class Test:
    def __init__(self):
        # Get parameters from the ROS parameter server
        self.depth_topic = rospy.get_param("/detector_node/depth_topic", "/default_depth_topic")
        
        # Create a cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the depth topic
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.test_callback)

    def test_callback(self, msg):
        # Conversion from ROS image message to OpenCV image
        rospy.loginfo("Received an image!")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

        # Loop through the image pixels and log the depth values
        for i in range(cv_image.shape[0]):
            for j in range(cv_image.shape[1]):
                value = cv_image[i, j]
                rospy.loginfo("Depth value at (%d, %d): %f", i, j, value)

        # Display the received image using OpenCV
        cv2.imshow("Received Image", cv_image)
        cv2.waitKey(30)

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("test_node")
    
    # Create an instance of the Test class
    test_instance = Test()

    # Keep the node running
    rospy.spin()