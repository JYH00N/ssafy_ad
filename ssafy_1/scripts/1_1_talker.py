#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    publisher = rospy.Publisher('ssafy', String, queue_size=10)
    rospy.init_node('ros_talker', anonymous=True)
    count = 0

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        hello_ssafy = "hello ssafy %s" % count
        rospy.loginfo(hello_ssafy)
        publisher.publish(hello_ssafy)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
