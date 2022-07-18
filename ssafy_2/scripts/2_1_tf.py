#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from morai_msgs.msg import EgoVehicleStatus
from math import pi

class Ego_listener():
    def __init__(self):
        rospy.init_node('status_listener', anonymous=True)
        
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.EgoStatus_callback)
        rospy.spin()

    def EgoStatus_callback(self,data): ## Ego Status subscriber
        self.status_msg=data
        print("tf broad cast")
        # 브로드캐스터 생성
        br = tf.TransformBroadcaster()
        # Ego 상태 tf 브로드캐스팅
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, 2),
                        tf.transformations.quaternion_from_euler(0,0, self.status_msg.heading/180*pi),
                        rospy.Time.now(),
                        "Ego",
                        "map")

if __name__ == '__main__':
    try:
        tl=Ego_listener()
    except rospy.ROSInternalException:
        pass
