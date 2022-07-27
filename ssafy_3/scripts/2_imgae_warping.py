#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


def warp_image(img, source_prop):
    
    image_size = (img.shape[1], img.shape[0])

    x = img.shape[1]
    y = img.shape[0]
    
    destination_points = np.float32([
    [0, y],
    [0, 0],
    [x, 0],
    [x, y]
    ])

    source_points = source_prop * np.float32([[x, y]]* 4)
    
    perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)
    
    warped_img = cv2.warpPerspective(img, perspective_transform, image_size, flags=cv2.INTER_LINEAR)
    
    return warped_img


class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        img_bgr = None

        self.source_prop = np.float32([[0.01, 0.80],
                                       [0.5 - 0.14, 0.52],
                                       [0.5 + 0.14, 0.52],
                                       [1 - 0.01, 0.80]
                                       ])

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_warp = warp_image(img_bgr, self.source_prop)

        img_concat = np.concatenate([img_bgr, img_warp], axis=1)

        # img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        # lower_wlane = np.array([0,0,180])
        # upper_wlane = np.array([30,70,255])

        # img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        # img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)

        # img_warp = warp_image(img_wlane, self.source_prop)

        # img_concat = np.concatenate([img_wlane, img_warp], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 


def main():

    rospy.init_node('lane_birdview', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()

if __name__ == '__main__':
    
    main()
