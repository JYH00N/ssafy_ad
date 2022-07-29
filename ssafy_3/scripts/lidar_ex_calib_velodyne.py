#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import time
import rospkg
import os
import json
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2

def translationMtx(x, y, z):
     
    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M


def rotationMtx(yaw, pitch, roll):
    
    R_x = np.array([[1,         0,              0,                0],
                    [0,         math.cos(roll), -math.sin(roll) , 0],
                    [0,         math.sin(roll), math.cos(roll)  , 0],
                    [0,         0,              0,               1],
                    ])
                     
    R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                    [0,                  1,      0               , 0],
                    [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                    [0,         0,              0,               1],
                    ])
    
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                    [math.sin(yaw),    math.cos(yaw),     0,    0],
                    [0,                0,                 1,    0],
                    [0,         0,              0,               1],
                    ])
                     
    R = np.matmul(R_x, np.matmul(R_y, R_z))
 
    return R

def transformMTX_lidar2cam(params_lidar, params_cam):

    #Relative position of lidar w.r.t cam
    lidar_pos = [params_lidar.get(i) for i in (["X","Y","Z"])]
    cam_pos = [params_cam.get(i) for i in (["X","Y","Z"])]

    x_rel = cam_pos[0] - lidar_pos[0]
    y_rel = cam_pos[1] - lidar_pos[1]
    z_rel = cam_pos[2] - lidar_pos[2]

    R_T = np.matmul(translationMtx(x_rel, y_rel, z_rel), rotationMtx(np.deg2rad(-90.), 0., 0.))
    R_T = np.matmul(R_T, rotationMtx(0, 0., np.deg2rad(-90.)))
    
    #rotate and translate the coordinate of a lidar
    R_T = np.linalg.inv(R_T)

    return R_T


def make_distance_img(xi, yi, distance, img_w, img_h, dis_max, clr_map):
    '''
    place the lidar points into numpy arrays in order to make distance map
    \n xi, yi : xy components of lidar points w.r.t a 2d normalized plane
    \n distance : distance measurement from the origin of the lidar coordinate
    \n img_w, img_h : a width and a height of a image from a camera
    \n dis_max : maximum of distance shown in the distance map 
    \n clr_map : colormap
    '''
    point_np = np.zeros((img_h,img_w,1), dtype=np.uint8)
    point_binary = np.zeros((img_h,img_w,3), dtype=np.uint8)

    point_np[yi.astype(np.int), xi.astype(np.int), :] = (np.clip(distance,0,dis_max).reshape([-1,1,1])/(dis_max)*255).astype(np.uint8)
    point_binary[yi.astype(np.int), xi.astype(np.int), :] = 1

    point_np = cv2.applyColorMap(point_np, clr_map)

    point_np = cv2.dilate(point_np*point_binary, cv2.getStructuringElement(cv2.MORPH_CROSS,(5, 5)))

    return point_np


def make_intensity_img(xi, yi, intens, img_w, img_h):
    '''
    place the lidar points into numpy arrays in order to make intensity map
    \n xi, yi : xy components of lidar points w.r.t a 2d normalized plane
    \n intens : intensities of lidar points
    \n img_w, img_h : a width and a height of a image from a camera
    '''
    point_np = np.zeros((img_h,img_w,3), dtype=np.uint8)

    #Object
    point_np[yi[intens>=250].astype(np.int),xi[intens>=250].astype(np.int),2] = 255
    
    return point_np



def project2img_mtx(params_cam):

    # focal lengths
    fc_x = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    #the center of image
    cx = params_cam["WIDTH"]/2
    cy = params_cam["HEIGHT"]/2
    
    #transformation matrix from 3D to 2D
    R_f = np.array([[fc_x,  0,      cx],
                    [0,     fc_y,   cy]])

    return R_f


class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):

        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)

        self.proj_mtx = project2img_mtx(params_cam)

    def transform_lidar2cam(self, xyz_p):
        
        xyz_c = np.matmul(np.concatenate([xyz_p, np.ones((xyz_p.shape[0], 1))], axis=1), self.RT.T)

        return xyz_c

    def project_pts2img(self, xyz_c, crop=True):

        xyz_c = xyz_c.T

        xc, yc, zc = xyz_c[0,:].reshape([1,-1]), xyz_c[1,:].reshape([1,-1]), xyz_c[2,:].reshape([1,-1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xyi = xyi[0:2,:].T

        if crop:
            xyi = self.crop_pts(xyi)
        else:
            pass
        
        return xyi

    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi

    
def draw_pts_img(img, xi, yi):

    point_np = img

    #Left Lane
    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)

    return point_np


class SensorCalib:
    
    def __init__(self):
    
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.scan_callback)

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)

        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.pc_np = None
        self.img = None

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):
        
        self.pc_np = self.pointcloud2_to_xyz(msg)

    def pointcloud2_to_xyz(self, cloud_msg):

        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):

            # print(point)

            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)

            if point[0] > 0 and dist < 10:
                point_list.append((point[0], point[1], point[2], dist))

        point_np = np.array(point_list, np.float32)

        return point_np

if __name__ == '__main__':
    
    rp = rospkg.RosPack()
    
    currentPath = rp.get_path("simple_detection_example")
    
    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    params_lidar = sensor_params["params_lidar"]

    rospy.init_node('ex_calib', anonymous=True)

    ex_calib_transform = SensorCalib()

    time.sleep(1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
            
        xyz_p = ex_calib_transform.pc_np[:, :3]

        xyz_c = ex_calib_transform.l2c_trans.transform_lidar2cam(xyz_p)

        xy_i = ex_calib_transform.l2c_trans.project_pts2img(xyz_c, crop=True)

        img_l2c = draw_pts_img(ex_calib_transform.img, xy_i[:, 0].astype(np.int32),
                                        xy_i[:, 1].astype(np.int32))
                                            
        cv2.imshow("Lidar2Cam", img_l2c)
        cv2.waitKey(1)

