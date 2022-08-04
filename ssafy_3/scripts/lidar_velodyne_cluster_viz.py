#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import PointCloud2, PointCloud
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray,Pose, Point32
from sklearn.cluster import DBSCAN

from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,ObjectStatus

class Cluster_viz:

    def __init__(self):

        rospy.Subscriber("/clusters", PoseArray, self.callback)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)

        self.object_pointcloud_pub = rospy.Publisher('object_pointcloud_data',PointCloud, queue_size=1)
        self.object_data_pub = rospy.Publisher('Object_topic_to_lidar',ObjectStatusList, queue_size=1)

        self.is_status = False
        self.cluster_status = False

        rate = rospy.Rate(30) # 10hz
        while not rospy.is_shutdown():
            if self.is_status == True and self.cluster_status == True:

                trans_matrix = np.array([
                                        [math.cos(self.vehicle_yaw), -math.sin(self.vehicle_yaw),self.vehicle_pos_x],
                                        [math.sin(self.vehicle_yaw),  math.cos(self.vehicle_yaw),self.vehicle_pos_y],
                                        [0,0,1]])

                obj_data=PointCloud()
                obj_data.header.frame_id='map'

                for i in self.cluster_data.poses :

                    local_result=np.array([[i.position.x], [i.position.y],[1]])
                    global_result=trans_matrix.dot(local_result)

                    tmp_point=Point32()
                    tmp_point.x = global_result[0][0]
                    tmp_point.y = global_result[1][0]
                    tmp_point.z = 1.
                    obj_data.points.append(tmp_point)

                self.object_pointcloud_pub.publish(obj_data)

            rate.sleep()

    def callback(self, msg):    
        self.cluster_data = msg

        self.cluster_status = True

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.status_msg=msg    

        self.vehicle_yaw = self.status_msg.heading/180*math.pi
        self.vehicle_pos_x = self.status_msg.position.x
        self.vehicle_pos_y = self.status_msg.position.y

        self.is_status = True



if __name__ == '__main__':

    rospy.init_node('velodyne_clustering', anonymous=True)

    Cluster_visualazation = Cluster_viz()

    rospy.spin() 
