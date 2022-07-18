#!/usr/bin/env python

import rospy
from morai_msgs.msg import EgoVehicleStatus

def EgoStatus_callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('Ego_status_listener', anonymous=True)

    rospy.Subscriber('/Ego_topic', EgoVehicleStatus, EgoStatus_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
