#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np

# camera_img 는 시뮬레이터에서 송신하는 Camera 센서 정보를 Subscriber 하는 예제 입니다.
# Camera 센서 정보인 /image_jpeg/compressed 라는 메세지를 Subscribe 합니다.
# Subscribe 한 데이터를 OpenCV 를 이용하여 Image 로 출력.

# 노드 실행 순서 
# 1. Callback 함수 생성 및 이미지 출력

#TODO: (1) Callback 함수 생성 및 데이터 출력
def Camera_callback(data):
    np_arr = np.fromstring(data.data, np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imshow("Image window", img_bgr)
    cv2.waitKey(1)

def listener():
    rospy.init_node('camera_img', anonymous=True)

    rospy.Subscriber('/image_jpeg/compressed', CompressedImage, Camera_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
