#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthImageSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('test_node', anonymous=True)

        # Create an instance of the CvBridge class
        self.bridge = CvBridge()

        # Subscribe to the depth image topic
        self.depth_image_subscriber = rospy.Subscriber('/iris/camera/depth/image_raw', Image, self.callback)
        #self.depth_image_subscriber = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.callback)


    def callback(self, msg):
        # Convert the ROS image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

        #cv2.imshow("Depth Image", cv_image)
        #cv2.waitKey(1)

        #cv_image = self.bridge.imgmsg_to_cv2(msg,"mono8")

        cv2.imshow("Depth Image", cv_image)
        cv2.waitKey(30)

        # Loop through the image pixels and log the depth values
        for i in range(cv_image.shape[0]):
            for j in range(cv_image.shape[1]):
                value = cv_image[i, j]
                #rospy.loginfo("Depth value at (%d, %d): %f", i, j, value)
                rospy.loginfo(value)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    depth_subscriber = DepthImageSubscriber()
    depth_subscriber.run()