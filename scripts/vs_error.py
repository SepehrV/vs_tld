#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tld_msgs.msg import Target, BoundingBox
from vs_tld.msg import Image_error
from vs_tld.msg import Error

class vs_error:
    def __init__ (self):
        rospy.init_node('vs_error', anonymous = True)
        self.sub1 = rospy.Subscriber("/cam1/image_error", Image_error, self.callback1)
        self.sub2 = rospy.Subscriber("/cam2/image_error", Image_error, self.callback2)
        self.pub_error = rospy.Publisher("image_error", Error, queue_size = 1)
        self.error = {
                'x1' : 0,
                'x2' : 0,
                'y1' : 0,
                'y2' : 0,
                'BB_size1' : 0,
                'BB_size2' : 0
                }

    def dic2float(self):
        return [self.error['x1'], self.error['y1'], self.error['BB_size1'], self.error['x2'], self.error['y2'], self.error['BB_size2']]

    def callback1(self, data):
        self.error['x1'] = data.ex
        self.error['y1'] = data.ey
        self.error['BB_size1'] = data.BB_size
        self.pub_error.publish(self.dic2float())
    def callback2(self, data):
        self.error['x2'] = data.ex
        self.error['y2'] = data.ey
        self.error['BB_size2'] = data.BB_size
        self.pub_error.publish(self.dic2float())


def main():
    E = vs_error()

if __name__=='__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "vs_error shutting down!"
