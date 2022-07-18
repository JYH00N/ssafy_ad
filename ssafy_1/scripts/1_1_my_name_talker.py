#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ssafy_1.msg import student

def talker():
    publisher = rospy.Publisher('my_name', student, queue_size=10)
    rospy.init_node('my_name_talker', anonymous=True)
    count = 0

    my_name = student()
    my_name.first_name = 'moon'
    my_name.last_name = 'daeyoung'
    my_name.age = 26
    my_name.score = 100

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        rospy.loginfo('\n my name : %s %s \n my age : %i \n SSAFY score : %i', my_name.first_name,my_name.last_name,my_name.age,my_name.score)
        publisher.publish(my_name)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
