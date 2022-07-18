#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

def imu_callback(data):
    rospy.loginfo("orientation:")
    rospy.loginfo("x : {} y : {} z : {} w : {}".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    rospy.loginfo("angular_velocity:")
    rospy.loginfo("x : {} y : {} z : {}".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    rospy.loginfo("linear_acceleration:")
    rospy.loginfo("x : {} y : {} z : {}".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    print("\n===============================================================\n")

def listener():
    rospy.init_node('imu_data_listener', anonymous=True)

    rospy.Subscriber('/imu', Imu, imu_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
