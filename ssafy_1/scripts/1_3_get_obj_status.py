#!/usr/bin/env python

import rospy
from morai_msgs.msg import ObjectStatusList

def Object_callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('Obj_status_listener', anonymous=True)

    rospy.Subscriber('/Object_topic', ObjectStatusList, Object_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
