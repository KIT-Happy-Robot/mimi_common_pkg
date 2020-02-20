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
import subprocess
#ROS関係ライブラリ
import rospy

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *

bc = BaseCarrier()

def main():
    rospy.loginfo('Start Test')
    #localizeObjectAC('person')
    approachPersonAC()
    rospy.loginfo('Finish Test')

if __name__ == '__main__':
    rospy.init_node('action_test', anonymous = True)
    main()
