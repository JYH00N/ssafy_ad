#!/usr/bin/env python

import rospy
from morai_msgs.msg import CollisionData

def Collision_callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('Collision_listener', anonymous=True)

    rospy.Subscriber('/CollisionData', CollisionData, Collision_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
