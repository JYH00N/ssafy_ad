#!/usr/bin/env python

import rospy
from morai_msgs.msg import GPSMessage

def gps_callback(data):
    print("\n ------------------------------------ \n")
    rospy.loginfo("latitude {}".format(data.latitude))
    rospy.loginfo("longitude {}".format(data.longitude))
    rospy.loginfo("eastOffset {}".format(data.eastOffset))
    rospy.loginfo("northOffset {}".format(data.northOffset))

def listener():
    rospy.init_node('gps_data_listener', anonymous=True)

    rospy.Subscriber('/gps', GPSMessage, gps_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
