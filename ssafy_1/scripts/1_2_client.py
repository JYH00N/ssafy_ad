#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ssafy_1.srv import AddTwoInts

def srv_client():
    rospy.init_node('ros_client', anonymous=True)
    rospy.wait_for_service('AddTwoInts')

    a = 10
    b = 11

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        try:
            AddTwoInts_srv = rospy.ServiceProxy('AddTwoInts', AddTwoInts)
            result = AddTwoInts_srv(a,b)
            rospy.loginfo('(Result) %i + %i = %i', a,b,result.sum)
        except rospy.ServiceException as e:
            rospy.logwarn('no respone')

        rate.sleep()

if __name__ == '__main__':
    try:
        srv_client()
    except rospy.ROSInterruptException:
        pass
