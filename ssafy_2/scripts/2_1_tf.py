#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from morai_msgs.msg import EgoVehicleStatus
from math import pi

# tf 는 물체의 위치와 자세 데이터를 좌표계로 나타내는 예제입니다.

# 노드 실행 순서 
# 1. Callback 함수 생성
# 2. 브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅

class Ego_listener():
    def __init__(self):
        rospy.init_node('status_listener', anonymous=True)
        
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.EgoStatus_callback)
        rospy.spin()

    #TODO: (1) Callback 함수 생성
    def EgoStatus_callback(self,data): ## Ego Status subscriber
        self.status_msg=data
        print("tf broad cast")

        #TODO: (2) 브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅
        br = tf.TransformBroadcaster()
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
