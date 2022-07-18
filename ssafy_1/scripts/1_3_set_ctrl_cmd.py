#!/usr/bin/env python

import rospy
from morai_msgs.msg import CtrlCmd

def talker():
    publisher = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
    rospy.init_node('Ego_Control_Command', anonymous=True)

    ctrl_cmd = CtrlCmd()
    ctrl_cmd.longlCmdType = 1
    ctrl_cmd.accel = 0.5
    ctrl_cmd.brake = 0.0
    ctrl_cmd.steering = 0.0

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        rospy.loginfo(ctrl_cmd)
        publisher.publish(ctrl_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
