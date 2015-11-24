#!/usr/bin/env python
import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tld_msgs.msg import Target, BoundingBox

def click_and_crop(event, x, y, flags, param):
    # grab references to the global variables
    global refPt, cropping, cropImage

    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt = [(x, y)]
        cropping = True
    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # record the ending (x, y) coordinates and indicate that
        # the cropping operation is finished
        refPt.append((x, y))
        cropping = False
        # draw a rectangle around the region of interest
        cv2.rectangle(cropImage, refPt[0], refPt[1], (0, 255, 0), 2)
        cv2.imshow("crop1", cropImage)
        msg = Target()
        msg.bb.x = refPt[0][0]
        msg.bb.y = refPt[0][1]
        msg.bb.width = x - refPt[0][0]
        msg.bb.height = y - refPt[0][1]
        msg.bb.confidence = 1
        msg.img = bridge.cv2_to_imgmsg(cropImage, 'bgr8')
        pub1 = rospy.Publisher("tld_gui_bb", Target, queue_size = 10)
        cv2.waitKey(1000)
        cv2.destroyWindow('crop1')
        pub1.publish(msg)

def subImages():
    global currentImage, cropImage
    rospy.init_node('gui')
    sub1 = rospy.Subscriber("/cam1/camera/image_raw", Image, showImage1)
    #sub2 = rospy.Subscriber("cam2", Image, showImage2)
    cv2.namedWindow('cam1')
    sub_tracker = rospy.Subscriber("/tld_tracked_object", BoundingBox, trackerPos)
    #cv2.namedWindow('cam2')
    shutdown = 0;
    while not shutdown:
        c = cv2.waitKey(10)
        if chr(c & 255)=='x':
            print 'shutting down!'
            shutdown = 1
            cv2.destroyAllWindows()
        if chr(c & 255) == 'c':
            cv2.namedWindow('crop1')
            I = currentImage.copy()
            cropImage = I
            cv2.moveWindow('crop1',300,300 )
            cv2.imshow('crop1', I);
            cv2.setMouseCallback("crop1", click_and_crop)

def showImage1(data):
    global currentImage
    try:
        cv_image = bridge.imgmsg_to_cv2(data, 'passthrough')
        b,g,r = cv2.split(cv_image)       # get b,g,r
        cv_image = cv2.merge([r,g,b])
        currentImage = cv_image
    except CvBridgeError, e:
        print e

    #cv2.imshow("cam1", cv_image)
    #cv2.waitKey(1)

def trackerPos(data):
    global currentImage, refPt
    refPt[0] = (data.x, data.y)
    refPt[1] = (data.x + data.height, data.y + data.width)
    cv2.rectangle(currentImage, refPt[0], refPt[1], (0, 255, 0), 2)
    cv2.imshow('cam1', currentImage)

def showImage2(self, data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError, e:
        print e

    cv2.imshow("cam2", cv_image)
    #cv2.waitKey(10)

if __name__ == '__main__':

    bridge = CvBridge()
    subImages()
