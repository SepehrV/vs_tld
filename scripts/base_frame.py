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
        rospy.init_node('vs_tld_gui', anonymous=True)
        #initialzing GUI variables
        self.bridge = CvBridge()
        self.currentImage = None
        self.cropping = False
        self.tracking = False
        self.command = 0
        self.ui_name = rospy.get_param('camera_number')
        #initializing camera topics and windows
        self.topic = rospy.get_param('camera_number') + rospy.get_param('~image_topic')
        self.sub = rospy.Subscriber(self.topic ,Image, self.showImage)
        cv2.namedWindow(self.ui_name, 1)
        cv2.moveWindow(self.ui_name, int(rospy.get_param('~window_pos')), 0)
        #initializng the trackers
        self.sub_tracker = rospy.Subscriber(rospy.get_param('camera_number') + "/tld_tracked_object", BoundingBox, self.trackerPos)
        self.pub_bb = rospy.Publisher(rospy.get_param('camera_number') + '/tld_gui_bb', Target, queue_size=0)

    def interface(self):
        if chr(self.command & 255) == 'c':
            if self.currentImage is not None:
                cv2.setMouseCallback(self.ui_name, self.click_and_crop)

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
        #cv2.imshow(self.ui_name, self.currentImage)

def base_frame():
    G = gui()
    #G.mainLoop()

if __name__ == '__main__':
    try:
        base_frame()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "base_frame shutting down"



