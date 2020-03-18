#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ActionClientを纏めたPythonスクリプト
# Author: Issei Iida
# Date: 2019/09/18
#--------------------------------------------------------------------

import sys

import rospy
import actionlib
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mimi_common_pkg.msg import (ApproachPersonAction, ApproachPersonGoal,
                                 EnterTheRoomAction, EnterTheRoomGoal,
                                 ExeActionPlanAction, ExeActionPlanGoal,
                                 LocalizeObjectAction, LocalizeObjectGoal)

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import m6Control


def approachPersonAC():
    rospy.loginfo("Start ApproachPerson")
    ac = actionlib.SimpleActionClient('approach_person', ApproachPersonAction)
    ac.wait_for_server()

    goal = ApproachPersonGoal()
    goal.data = 'start'

    ac.send_goal(goal)
    ac.wait_for_result()

    result = ac.get_result()
    if result.data:
        rospy.loginfo('Success ApproachPerson')
        return True
    else:
        rospy.loginfo('Failed ApproachPerson')
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
        return True
    else:
        rospy.loginfo("Failed EnterTheRoom")
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
    if result == 'success':
        rospy.loginfo("Success ExeActionPlan")
        return True
    else:
        rospy.loginfo("Failed ExeActionPlan")
        return False


def localizeObjectAC(receive_msg):
    rospy.loginfo('Start LocalizeObject')
    ac = actionlib.SimpleActionClient('localize_object', LocalizeObjectAction)
    ac.wait_for_server()

    goal = LocalizeObjectGoal()
    goal.data = receive_msg

    ac.send_goal(goal)
    ac.wait_for_result()

    result = ac.get_result()
    if result:
        rospy.loginfo('Success LocalizeObject')
        return True
    else:
        rospy.loginfo('Failed LocalizeObject')
        return False


def navigationAC(coord_list):
    rospy.loginfo("Start Navigation")
    m6Control(0.0)
    ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    ac.wait_for_server()
    clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = coord_list[0]
    goal.target_pose.pose.position.y = coord_list[1]
    goal.target_pose.pose.orientation.z = coord_list[2]
    goal.target_pose.pose.orientation.w = coord_list[3]

    clear_costmaps()
    rospy.wait_for_service('move_base/clear_costmaps')
    rospy.sleep(0.3)
    ac.send_goal(goal)
    state = ac.get_state()
    count = 0 # clear_costmapsの実行回数をカウンタ
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
            if count == 3:
                count = 0
                rospy.loginfo('Navigation Failed')
                return False
            else:
                rospy.loginfo('Clearing Costmaps')
                clear_costmaps()
                ac.send_goal(goal)
                rospy.loginfo('Send Goal')
                count += 1
