#!/usr/bin/env python3

import os
import rospy
import cv2
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub = rospy.Subscriber('image', Image, self.callback)
        self.bridge = CvBridge()

    def callback(self, ros_img):
        rospy.loginfo(ros_img.header)
        cv_image = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
        #cv2.imshow('Frame',cv_image)

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()
