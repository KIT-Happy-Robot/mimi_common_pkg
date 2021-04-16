#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: デバッグ用ROSノード
# Author: Issei Iida
# Date: 2019/10/11
#---------------------------------------------------------------------

import sys
import rospy
sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_voice_control/src')
# from voice_common_pkg.srv import WhatDidYouSay
 
sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *

def main():
    # WDYS = rospy.ServiceProxy('/bf/conversation_srvserver', WhatDidYouSay)
    rospy.loginfo('Start Test')
    # result = WDYS().result
    action = ['go', 'grasp', 'go', 'give']
    data = ['table', 'red cup', 'operator', 'red cup']
    # action = ['go', 'grasp', 'go', 'place']
    # data = ['cupboard', 'red cup', 'table', 'red cup']
    result = exeActionPlanAC(action, data)
    
    # rospy.loginfo('Start Test')
    # localizeObjectAC('person')
    # approachPersonAC()
    rospy.loginfo('Finish Test')

if __name__ == '__main__':
    rospy.init_node('action_test', anonymous = True)
    main()
