#!/usr/bin/env python3

import os
import rospy
import cv2
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VirtCamera(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(VirtCamera, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # construct publisher
        self.pub = rospy.Publisher('image', Image)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/code/catkin_ws/src/draw-line-crossroad/assets/video.mp4')
 
        # Check if camera opened successfully
        if (self.cap.isOpened()== False): 
            rospy.loginfo("Error opening video stream or file")
        

    def run(self):
        # create loop
        rate = rospy.Rate(1) # 1Hz
        
        while not rospy.is_shutdown():
            ret, cv_img = self.cap.read()
            if ret == False:
                rospy.loginfo("Error get image from video cap")
                break
            
            #publish img
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = VirtCamera(node_name='virt_camera_1')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
