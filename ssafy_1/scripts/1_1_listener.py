#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo('%s', data.data)

def listener():
    rospy.init_node('ros_listener', anonymous=True)

    rospy.Subscriber('ssafy', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
