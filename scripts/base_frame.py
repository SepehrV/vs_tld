#!/usr/bin/env python
import rospy
import math
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tld_msgs.msg import Target, BoundingBox
from vs_tld.msg import Image_error

class gui:
    def __init__ (self):
        #initialzing the node
        rospy.init_node('vs_tld_gui', anonymous=True)
        #initialzing GUI variables
        self.bridge = CvBridge()
        self.currentImage = None
        self.cropping = False
        self.tracking = False
        self.command = 0
        self.ui_name = rospy.get_param('camera_number')
        self.image_size = None
        #initializing camera topics and windows
        self.topic = rospy.get_param('camera_number') + rospy.get_param('~image_topic')
        self.sub = rospy.Subscriber(self.topic ,Image, self.showImage)
        self.pub_error = rospy.Publisher(rospy.get_param('camera_number')+'/image_error', Image_error, queue_size=1)
        cv2.namedWindow(self.ui_name, 1)
        cv2.moveWindow(self.ui_name, int(rospy.get_param('~window_pos')), 0)
        #initializng the trackers
        self.sub_tracker = rospy.Subscriber(rospy.get_param('camera_number') + "/tld_tracked_object", BoundingBox, self.trackerPos)
        self.pub_bb = rospy.Publisher(rospy.get_param('camera_number') + '/tld_gui_bb', Target, queue_size=0)
        self.record_desired = None

    def interface(self):
        if chr(self.command & 255) == 'c':
            if self.currentImage is not None:
                cv2.setMouseCallback(self.ui_name, self.click_and_crop)
        if chr(self.command & 255) == 'r':
            self.record_desired = [self.track_data.x,self.track_data.y,math.sqrt(float(self.track_data.height)*float(self.track_data.width))]

    def click_and_crop(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.cropping = True
            self.refPt = [(x,y)]

        if event == cv2.EVENT_LBUTTONUP:
            self.refPt.append((x,y))
            cv2.rectangle(self.currentImage, self.refPt[0], self.refPt[1], (0,255,0), 2)
            msg = Target()
            msg.bb.x = self.refPt[0][0]
            msg.bb.y = self.refPt[0][1]
            msg.bb.width = x - self.refPt[0][0]
            msg.bb.height = y - self.refPt[0][1]
            msg.bb.confidence = 1
            msg.img = self.bridge.cv2_to_imgmsg(self.currentImage, 'bgr8')
            self.pub_bb.publish(msg)
            cv2.imshow(self.ui_name, self.currentImage)
            cv2.waitKey(1000)
            cv2.destroyWindow(self.ui_name)
            cv2.namedWindow(self.ui_name, 1)
            cv2.moveWindow(self.ui_name, int(rospy.get_param('~window_pos')), 0)
            self.cropping = False
            self.tracking = True

    def showImage(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.currentImage = cv_image
            if self.image_size == None:
                self.image_size = self.currentImage.shape
        except CvBridgeError, e:
            print e
        if self.tracking:
            pt1 = (self.track_data.x, self.track_data.y)
            pt2 = (self.track_data.x+self.track_data.width, self.track_data.y+self.track_data.height)
            cv2.rectangle(self.currentImage, pt1, pt2, (0, 255, 0), 2)

        if self.cropping is False:
            cv2.imshow(self.ui_name, cv_image)
        self.command = cv2.waitKey(3)
        self.interface()

    def trackerPos(self, data):
        self.track_data = data
        self.calc_error(data)

    def calc_error(self, data):
        if self.record_desired is not None:
            self.error = (float((data.x - self.record_desired[0]))/self.image_size[1], float((data.y - self.record_desired[1]))/self.image_size[0],(math.sqrt(float(data.height)*float(data.width) - self.record_desired[2])/float((self.image_size[0])*float(self.image_size[1]))))
        else:
            self.error = ((data.x - float(self.image_size[1])/2)/self.image_size[1], (data.y - float(self.image_size[0])/2)/float(self.image_size[0]), 0.25 - math.sqrt(float(data.height)*float(data.width)/float((self.image_size[0])*float(self.image_size[1]))) )
        msg = Image_error()
        msg.ex = self.error[0]
        msg.ey = self.error[1]
        msg.BB_size = self.error[2]
        self.pub_error.publish(msg)
def base_frame():
    G = gui()

if __name__ == '__main__':
    try:
        base_frame()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "base_frame shutting down"



