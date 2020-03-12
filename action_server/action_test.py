#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
#Title: ActionServerNodeのデバッグ用ROSノード
#Author: Issei Iida
#Date: 2019/10/11
#Memo:
#---------------------------------------------------------------------

#Python関係ライブラリ
import sys
#ROS関係ライブラリ
import rospy

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *
from dynamic_reconfigure.msg import Config


class unko():
    def __init__(self):
        self.xy_goal_tolerance_pub = rospy.Publisher('/move_base/DWAPlannerROS/parameter_update',
                                                     Config,
                                                     queue_size = 1)
        self.reconfig_data = Config()

    def uuu(self):
        print self.reconfig_data.doubles
        self.reconfig_data.doubles = [{'xy_goal_tolerance':'0.8'}]
        self.xy_goal_tolerance_pub.publish(self.reconfig_data)

def main():
    rospy.loginfo('Start Test')
    # localizeObjectAC('person')
    approachPersonAC()
    rospy.loginfo('Finish Test')

if __name__ == '__main__':
    rospy.init_node('action_test', anonymous = True)
    # x = unko()
    # x.uuu()
    main()
