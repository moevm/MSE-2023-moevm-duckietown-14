#!/usr/bin/env python3

import os
import rospy
import cv2

from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from LaneDetector.IntersectionController import *
from LaneDetector.EdgeKernels import *

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # Setting up parameters
        self.turn_direct = DTParam(
            'turn_direct',
            param_type=ParamType.STRING
        )

        # construct publisher
        self.pub = rospy.Publisher('~image_drawing_line', Image)
        # construct subscriber
        self.sub = rospy.Subscriber('image', Image, self.callback)
        self.bridge = CvBridge()

    def callback(self, ros_img):
        str_direct_turn = self.turn_direct.value

        # Log turn_direct
        if str_direct_turn == "l":
            rospy.loginfo("Turn left")
        elif str_direct_turn == "u":
            rospy.loginfo("Drive straight")
        elif str_direct_turn == "r":
            rospy.loginfo("Turn right")
        else : 
            rospy.loginfo("Incorrect data: Drive straight")

        #rospy.loginfo(ros_img.header)
        cv_image = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")


        IC = IntersectionController(scharr)
        self.pub.publish(IC.renderImage(cv_image))
        

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()
