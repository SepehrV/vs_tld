#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tld_msgs.msg import Target, BoundingBox
from base_frame import gui


if __name__ == '__main__':
    G = gui()
    G.mainLoop()
