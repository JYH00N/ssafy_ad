#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from morai_msgs.msg import ScenarioLoad
from morai_msgs.srv import MoraiScenarioLoadSrv

def srv_client():
    rospy.init_node('ros_client', anonymous=True)
    rospy.wait_for_service('/Service_MoraiSL')

    scenario_setting = ScenarioLoad()
    scenario_setting.file_name                      = "ssafy_scenario"
    scenario_setting.load_network_connection_data   = False
    scenario_setting.delete_all                     = False
    scenario_setting.load_ego_vehicle_data          = False
    scenario_setting.load_surrounding_vehicle_data  = True
    scenario_setting.load_pedestrian_data           = True
    scenario_setting.load_obstacle_data             = True
    scenario_setting.set_pause                      = False

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        try:
            ros_srv = rospy.ServiceProxy('/Service_MoraiSL', MoraiScenarioLoadSrv)
            result = ros_srv(scenario_setting)
            rospy.loginfo(result)
        except rospy.ServiceException as e:
            rospy.logwarn('no respone')

        rate.sleep()

if __name__ == '__main__':
    try:
        srv_client()
    except rospy.ROSInterruptException:
        pass
