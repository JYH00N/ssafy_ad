#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ssafy_1.msg import student

def callback(data):
    rospy.loginfo('\n my name : %s %s \n my age : %i \n SSAFY score : %i', data.first_name,data.last_name,data.age,data.score)

def listener():
    rospy.init_node('my_name_listener', anonymous=True)

    rospy.Subscriber('my_name', student, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
