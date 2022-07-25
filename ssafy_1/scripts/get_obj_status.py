#!/usr/bin/env python

import rospy
from morai_msgs.msg import ObjectStatusList

# Obj_status_listener 는 시뮬레이터에서 송신하는 Object 정보를 Subscriber 하는 예제 입니다.
# 시뮬레이터 내 Object 정보인 /Object_topic 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력
def Object_callback(data):
    rospy.loginfo(data)

def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('Obj_status_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    rospy.Subscriber('/Object_topic', ObjectStatusList, Object_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
