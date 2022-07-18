#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
import numpy as np
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import ObjectStatus, ObjectStatusList, EgoVehicleStatus

class lc_path_pub :
    def __init__(self):
        rospy.init_node('lc_path_pub', anonymous=True)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)
        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1)
        self.lc_1=Path()
        self.lc_1.header.frame_id='/map'
        self.lc_2=Path()
        self.lc_2.header.frame_id='/map'

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('ssafy_2')
        lc_1 = pkg_path+'/path'+'/lc_1.txt'
        self.f=open(lc_1,'r')
        lines=self.f.readlines()
        for line in lines :
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.orientation.w=1
            self.lc_1.poses.append(read_pose)        
        self.f.close()

        lc_2 = pkg_path+'/path'+'/lc_2.txt'
        self.f=open(lc_2,'r')
        lines=self.f.readlines()
        for line in lines :
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.orientation.w=1
            self.lc_2.poses.append(read_pose)        
        self.f.close()

        self.is_object_info = False
        self.is_status = False

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.is_object_info == True and self.is_status == True:
                global_obj,local_obj=self.calc_vaild_obj([self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi])
                global_path = self.lc_1
                self.global_path_pub.publish(global_path)

            rate.sleep()

    def statusCB(self,data): ## Vehicle Status Subscriber 
        self.status_msg = data
        self.is_status = True

    def object_info_callback(self,data): ## Object information Subscriber
        self.is_object_info = True
        self.object_num=data.num_of_npcs
        object_type=[]
        object_pose_x=[]
        object_pose_y=[]
        object_velocity=[]
        for num in range(data.num_of_npcs) :
            object_type.append(data.npc_list[num].type)
            object_pose_x.append(data.npc_list[num].position.x)
            object_pose_y.append(data.npc_list[num].position.y)
            object_velocity.append(data.npc_list[num].velocity.x)

        self.object_info=[object_type,object_pose_x,object_pose_y,object_velocity]

    def calc_vaild_obj(self,ego_pose):  # x, y, heading
        global_object_info=[]
        loal_object_info=[]

        tmp_theta=ego_pose[2]
        tmp_translation=[ego_pose[0],ego_pose[1]]
        tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],
                        [sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],
                        [0,0,1]])
        tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],
                            [tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],
                            [0,0,1]])

        for num in range(self.object_num):
            global_result=np.array([[self.object_info[1][num]],[self.object_info[2][num]],[1]])
            local_result=tmp_det_t.dot(global_result)
            if local_result[0][0]> 0 :
                global_object_info.append([self.object_info[0][num],self.object_info[1][num],self.object_info[2][num],self.object_info[3][num]])
                loal_object_info.append([self.object_info[0][num],local_result[0][0],local_result[1][0],self.object_info[3][num]])
        

        return global_object_info,loal_object_info

if __name__ == '__main__':
    try:
        test_track=lc_path_pub()
    except rospy.ROSInterruptException:
        pass
