#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ActionClientを纏めたPythonスクリプト
# Author: Issei Iida
# Date: 2019/09/18
#--------------------------------------------------------------------

# Python
import sys
# ROS
import rospy
import actionlib
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mimi_common_pkg.msg import *

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import *


def approachPersonAC():
    rospy.loginfo("Start ApproachPerson")
    ac = actionlib.SimpleActionClient('approach_person', ApproachPersonAction)
    ac.wait_for_server()

    goal = ApproachPersonGoal()
    goal.data = 'start'

    ac.send_goal(goal)
    ac.wait_for_result()

    result = ac.get_result()
    if result.data == 'success':
        rospy.loginfo('Success ApproachPerson')
        ac.cancel_goal()
        return True
    else:
        rospy.loginfo('Failed ApproachPerson')
        ac.cancel_goal()
        return False


def enterTheRoomAC(receive_msg):
    rospy.loginfo("Start EnterTheRoom")
    ac = actionlib.SimpleActionClient('enter_the_room', EnterTheRoomAction)
    ac.wait_for_server()

    goal = EnterTheRoomGoal()
    goal.distance = receive_msg

    ac.send_goal(goal)
    ac.wait_for_result()

    result = ac.get_result()
    if result.data:
        rospy.loginfo("Success EnterTheRoom")
        # ac.cancel_goal()
        return True
    else:
        rospy.loginfo("Failed EnterTheRoom")
        # ac.cancel_goal()
        return False


def exeActionPlanAC(action_list, data_list):
        ac = actionlib.SimpleActionClient('exe_action_plan', ExeActionPlanAction)
        ac.wait_for_server()

        goal = ExeActionPlanGoal()
        goal.action = action_list
        goal.data = data_list

        ac.send_goal(goal)
        ac.wait_for_result()

        result = ac.get_result().data
        print result
        if result == 'success':
            rospy.loginfo("Success ExeActionPlan")
            result = 'none'
            # ac.cancel_goal()
            return True
        else:
            rospy.loginfo("Failed ExeActionPlan")
            # ac.cancel_goal()
            return False


def localizeObjectAC(receive_msg):
    rospy.loginfo('Start LocalizeObject')
    ac = actionlib.SimpleActionClient('localize_object', LocalizeObjectAction)
    ac.wait_for_server()

    goal = LocalizeObjectGoal()
    goal.data = receive_msg
    print goal.data

    ac.send_goal(goal)
    ac.wait_for_result()

    result = ac.get_result()
    if result:
        rospy.loginfo('Success LocalizeObject')
        #ac.cancel_goal()
        return True
    else:
        rospy.loginfo('Failed LocalizeObject')
        # ac.cancel_goal()
        return False


def navigationAC(coord_list):
    rospy.loginfo("Start Navigation")
    m6Control(-0.2)
    ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    ac.wait_for_server()
    # Service
    clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    # Set Goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = coord_list[0]
    goal.target_pose.pose.position.y = coord_list[1]
    goal.target_pose.pose.orientation.z = coord_list[2]
    goal.target_pose.pose.orientation.w = coord_list[3]
    # Costmapを消去
    clear_costmaps()
    rospy.wait_for_service('move_base/clear_costmaps')
    rospy.sleep(0.3)
    ac.send_goal(goal)
    state = ac.get_state()
    count = 0# <---clear_costmapsの実行回数をカウントするための変数
    while not rospy.is_shutdown():
        state = ac.get_state()
        if state == 1:
            rospy.loginfo('Running...')
            rospy.sleep(1.0)
        elif state == 3:
            rospy.loginfo('Navigation success!!')
            state = 0
            return True
        elif state == 4:
            if count == 5:
                count = 0
                rospy.loginfo('Navigation Failed')
                return False
            else:
                rospy.loginfo('Clear Costmaps')
                clear_costmaps()
                ac.send_goal(goal)
                rospy.loginfo('Send Goal')
                count += 1
