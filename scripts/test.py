#!/usr/bin/env python
import rospy
from std_msgs.msg import String
def talker():
    pub = rospy.Publisher('chatter', String, queue_size = 10)
    rospy.init_node('talker')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        s = 'sepehr' + str(rospy.get_time())
        rospy.loginfo(s)
        pub.publish(s)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
