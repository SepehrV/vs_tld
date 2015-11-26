#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tld_msgs.msg import Target, BoundingBox

class gui:
    def __init__ (self):
        #initialzing the node
        rospy.init_node('vs_tld_gui')

        #initializing camera topics and windows
        self.sub1 = rospy.Subscriber("/cam1/camera/image_raw", Image, self.showImage1)
        self.sub2 = rospy.Subscriber("/cam2/camera/image_raw", Image, self.showImage2)
        cv2.namedWindow("cam1")
        cv2.namedWindow("cam2")
        cv2.moveWindow('cam2',700,0)
        #initializng the trackers
        self.sub_tracker1 = rospy.Subscriber("/cam1/tld_tracker_object", BoundingBox, self.trackerPos1)
        self.sub_tracker2 = rospy.Subscriber("/cam2/tld_tracker_object", BoundingBox, self.trackerPos2)
        self.pub_bb1 = rospy.Publisher('/cam1/tld_gui_bb', Target, queue_size=10)
        self.pub_bb2 = rospy.Publisher('/cam2/tld_gui_bb', Target, queue_size=10)
        #initialzing GUI variables
        self.shutdown = 0
        self.rate = rospy.Rate(30)
        self.bridge = CvBridge()
        self.currentImage1 = None
        self.currentImage2 = None
        self.cropping = False
        self.command = 0
        self.show1_ready = True
    def mainLoop(self):
        while not self.shutdown:
            #self.command = cv2.waitKey(10)
            if chr(self.command & 255) == 'x':
                print "Shutting Down!"
                self.shutdown = 1
                cv2.destroyAllWindows()

            if chr(self.command & 255) == 'c':
                if self.currentImage1 is not None:
                    cv2.setMouseCallback("cam1", self.click_and_crop, param = 1)
                if self.currentImage2 is not None:
                    cv2.setMouseCallback('cam2', self.click_and_crop, param = 2)

    def click_and_crop(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.cropping = True
            if param == 1:
                self.refPt1 = [(x,y)]
            else:
                self.refPt2 = [(x,y)]
        if event == cv2.EVENT_LBUTTONUP:
            if param == 1:
                self.refPt1.append((x,y))
                cv2.rectangle(self.currentImage1, self.refPt1[0], self.refPt1[1], (0,255,0), 2)
                msg = Target()
                msg.bb.x = self.refPt1[0][0]
                msg.bb.y = self.refPt1[0][1]
                msg.bb.width = x - self.refPt1[0][0]
                msg.bb.height = y - self.refPt1[0][1]
                msg.bb.confidence = 1
                msg.img = self.bridge.cv2_to_imgmsg(self.currentImage1, 'bgr8')
                self.pub_bb1.publish(msg)
                cv2.imshow('cam1', self.currentImage1)
                cv2.waitKey(1000)
            else:
                self.refPt2.append((x,y))
                cv2.rectangle(self.currentImage1, self.refPt2[0], self.refPt2[1], (0,255,0), 2)
                msg = Target()
                msg.bb.x = self.refPt2[0][0]
                msg.bb.y = self.refPt2[0][1]
                msg.bb.width = x - self.refPt2[0][0]
                msg.bb.height = y - self.refPt2[0][1]
                msg.bb.confidence = 1
                msg.img = self.bridge.cv2_to_imgmsg(self.currentImage2, 'bgr8')
                self.pub_bb2.publish(msg)
                cv2.imshow('cam2', self.currentImage2)
                cv2.waitKey(1000)

            self.cropping = False

    def showImage1(self, data):
        if self.show1_ready == False:
            return
        self.show1_ready = False
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
            b,g,r = cv2.split(cv_image)       # get b,g,r
            cv_image = cv2.merge([r,g,b])
            self.currentImage1 = cv_image
        except CvBridgeError, e:
            print e
        if self.cropping is False:
            cv2.imshow('cam1', cv_image)
            self.command = cv2.waitKey(20)
            self.show1_ready = True

    def showImage2(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
            b,g,r = cv2.split(cv_image)       # get b,g,r
            cv_image = cv2.merge([r,g,b])
            self.currentImage2 = cv_image
        except CvBridgeError, e:
            print e
        if self.cropping is False:
            cv2.imshow('cam2', cv_image)
            self.command = cv2.waitKey(20)

    def trackerPos1(self, data):
        self.refPt1[0] = (data.x, data.y)
        self.refPt1[1] = (data.x + data.height, data.y + data.width)
        cv2.rectangle(currentImage1, refPt1[0], refPt1[1], (0, 255, 0), 2)
        cv2.imshow('cam1', currentImage1)


    def trackerPos2(self, data):
        self.refPt2[0] = (data.x, data.y)
        self.refPt2[1] = (data.x + data.height, data.y + data.width)
        cv2.rectangle(currentImage2, refPt2[0], refPt2[1], (0, 255, 0), 2)
        cv2.imshow('cam1', currentImage2)



