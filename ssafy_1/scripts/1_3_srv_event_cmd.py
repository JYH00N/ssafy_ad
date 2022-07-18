#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from morai_msgs.msg import EventInfo,Lamps
from morai_msgs.srv import MoraiEventCmdSrv

def srv_client():
    rospy.init_node('ros_client', anonymous=True)
    rospy.wait_for_service('/Service_MoraiEventCmd')

    lamp_cmd = Lamps()
    lamp_cmd.turnSignal = 1
    lamp_cmd.emergencySignal = 0
        
    set_Event_control = EventInfo()
    set_Event_control.option = 7
    set_Event_control.ctrl_mode = 3
    set_Event_control.gear = 4
    set_Event_control.lamps = lamp_cmd

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        try:
            ros_srv = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
            result = ros_srv(set_Event_control)
            rospy.loginfo(result)
        except rospy.ServiceException as e:
            rospy.logwarn('no respone')

        rate.sleep()

if __name__ == '__main__':
    try:
        srv_client()
    except rospy.ROSInterruptException:
        pass
