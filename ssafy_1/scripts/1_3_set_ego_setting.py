#!/usr/bin/env python

import rospy
from morai_msgs.msg import MultiEgoSetting

def talker():
    publisher = rospy.Publisher('/ego_setting', MultiEgoSetting, queue_size=10)
    rospy.init_node('Ego_setting_Command', anonymous=True)

    ego_setting_msg = MultiEgoSetting()
    ego_setting_msg.number_of_ego_vehicle=1
    ego_setting_msg.camera_index = 0
    ego_setting_msg.ego_index = [0]
    ego_setting_msg.global_position_x = [13.4]
    ego_setting_msg.global_position_y = [1099,68]
    ego_setting_msg.global_position_z = [3]
    ego_setting_msg.global_roll = [0]
    ego_setting_msg.global_pitch = [0]
    ego_setting_msg.global_yaw = [60.0]
    ego_setting_msg.velocity=[0]
    ego_setting_msg.gear=[4]
    ego_setting_msg.ctrl_mode=[16] 

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        rospy.loginfo(ego_setting_msg)
        publisher.publish(ego_setting_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
