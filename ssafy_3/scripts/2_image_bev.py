#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from camera_utils import BEVTransform

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.image_bgr = None

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.image_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        # cv2.imshow("Image window", self.image_bgr)
        # cv2.waitKey(1) 

def main():

    rospack = rospkg.RosPack()
    pkg_path=rospack.get_path("ssafy_3")
    
    with open(os.path.join(pkg_path, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        if image_parser.image_bgr is not None:

            img_warp = bev_op.warp_bev_img(image_parser.image_bgr)

            cv2.imshow("Image window", img_warp)
            cv2.waitKey(1)

            rate.sleep()

if __name__ == '__main__':
    
    main()