#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ssafy_1.srv import AddTwoInts,AddTwoIntsResponse

def add_two_ints(req):
    rospy.loginfo('(AddTwoInts) %i + %i = ', req.a,req.b)
    return AddTwoIntsResponse(req.a + req.b)

def srv_server():
    rospy.init_node('AddTwoInts_server', anonymous=True)

    s = rospy.Service('AddTwoInts', AddTwoInts, add_two_ints)

    rospy.spin()

if __name__ == '__main__':
    srv_server()
