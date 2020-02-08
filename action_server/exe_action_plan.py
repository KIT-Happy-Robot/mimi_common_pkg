#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ActionPlanから行動を決定して実行するアクションサーバー
# Author: Issei Iida
# Date: 2020/02/07
# Memo:
#--------------------------------------------------------------------

# Python
import sys
# ROS
import rospy
import rosparam
import actionlib
import smach
import smach_ros
from smach import StateMachine
from smach_ros import ActionServerWrapper
# Message/Service
from mimi_common_pkg.msg import *
from std_msgs.msg import String, Bool
from mimi_common_pkg.srv import ManipulateSrv

sys.path.insert(0, '/home/issei/catkin_ws/src/mimi_common_pkg/scripts/')
# from common_action_client import *
from common_function import *


class determineAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['move', 'mani', 'search', 'speak', 'all_finish'],
                             input_keys = ['goal_in', 'a_num_in', 'result_message'],
                             output_keys = ['a_action_out', 'a_data_out',
                                            'a_num_out', 'result_message'])
        # Param
        self.action_state = rosparam.get_param('/action_state')

    def execute(self, userdata):
        rospy.loginfo('Executing state: DETERMINE_ACTION')
        a_count = userdata.a_num_in
        a_plan = userdata.goal_in
        print str(a_count)
        if a_count < len(a_plan.action):
            userdata.a_action_out = a_plan.action[a_count]
            userdata.a_data_out = a_plan.data[a_count]
            a_name = a_plan.action[a_count]
            return self.action_state[a_name]
        else:
            rospy.loginfo('all success')
            # userdata.goal_out.action = []
            # userdata.goal_out.data = []
            userdata.a_num_out = 0
            userdata.result_message.data = 'success'
            return 'all_finish'


class move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['move_finish', 'move_failed'],
                             input_keys = ['action_in', 'data_in', 'num_in'],
                             output_keys = ['a_num_out'])
        # Publisher
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)
 
    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        a_count = userdata.num_in
        name = userdata.action_in
        data = userdata.data_in
        print data
        if name == 'go':
            # coord_list = searchLocationName('location_dict', data)
            self.pub_location.publish(data)
            # speak('I move to ' + data)
            # result = navigationAC(coord_list)
            result = True
            if result:
                userdata.a_num_out = a_count + 1 
                return 'move_finish'
            else:
                userdata.a_num_out = 0 
                return 'move_failed'
        else:
            return 'move_finish'


class mani(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['mani_finish', 'mani_failed'],
                             input_keys = ['action_in', 'data_in', 'num_in'],
                             output_keys = ['a_num_out'])
        # Service
        self.grasp_srv = rospy.ServiceProxy('/manipulation', ManipulateSrv)
        self.arm_srv = rospy.ServiceProxy('/change_arm_pose', ManipulateSrv)
        # Param
        self.object_list = rosparam.get_param('/object_mapping')
 
    def execute(self, userdata):
        rospy.loginfo('Executing state: MANI')
        a_count = userdata.num_in
        name = userdata.action_in
        data = userdata.data_in
        if name == 'grasp':
            print 'grasp'
            obj = self.object_list[data]
            print obj
            # result = self.grasp_srv(obj).result
            result = True
        elif name == 'place':
            print 'place'
            # result == self.arm_srv(name).result
            result = True
        elif name == 'give':
            print 'give'
            # result = self.arm_srv(name).result
            result = True
        rospy.loginfo('Result is ' + str(result))
        if result:
            userdata.a_num_out = a_count + 1 
            return 'mani_finish'
        else:
            userdata.a_num_out = 0 
            return 'mani_failed'


class search(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['search_finish', 'search_failed'],
                             input_keys = ['action_in', 'data_in', 'num_in'],
                             output_keys = ['a_num_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: SEARCH')
        a_count = userdata.num_in
        data = userdata.data_in
        # result = localizeObjectAC(data)
        result = True
        if result:
            userdata.a_num_out = a_count + 1 
            return 'search_finish'
        else:
            userdata.a_num_out = 0 
            return 'search_failed'


class speak(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['speak_finish', 'speak_failed'],
                             input_keys = ['action_in', 'data_in', 'num_in'],
                             output_keys = ['a_num_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: SPEAK')
        a_count = userdata.num_in
        data = userdata.data_in
        print data
        # speak(data)
        userdata.a_num_out = a_count + 1 
        return 'speak_finish'


def main():
    sm_top = StateMachine(
            outcomes = ['success',
                        'action_failed',
                        'preempted'],
            input_keys = ['goal_message',
                          'result_message'],
            output_keys = ['result_message'])
    sm_top.userdata.action_num = 0
    with sm_top:
        StateMachine.add(
                'DETERMINE_ACTION',
                determineAction(),
                transitions = {'move':'MOVE',
                               'mani':'MANI',
                               'search':'SEARCH',
                               'speak':'SPEAK',
                               'all_finish':'success'},
                remapping = {'goal_in':'goal_message',
                             'a_num_in':'action_num',
                             'a_action_out':'action_name',
                             'a_data_out':'data_name',
                             'a_num_out':'action_num',
                             'result_out':'result_message'})

        StateMachine.add(
                'MOVE',
                move(),
                transitions = {'move_finish':'DETERMINE_ACTION',
                               'move_failed':'action_failed'},
                remapping = {'action_in':'action_name',
                             'data_in':'data_name',
                             'num_in':'action_num',
                             'a_num_out':'action_num'})

        StateMachine.add(
                'MANI',
                mani(),
                transitions = {'mani_finish':'DETERMINE_ACTION',
                               'mani_failed':'action_failed'},
                remapping = {'action_in':'action_name',
                             'data_in':'data_name',
                             'num_in':'action_num',
                             'a_num_out':'action_num'})

        StateMachine.add(
                'SEARCH',
                search(),
                transitions = {'search_finish':'DETERMINE_ACTION',
                               'search_failed':'action_failed'},
                remapping = {'action_in':'action_name',
                             'data_in':'data_name',
                             'num_in':'action_num',
                             'a_num_out':'action_num'})

        StateMachine.add(
                'SPEAK',
                speak(),
                transitions = {'speak_finish':'DETERMINE_ACTION',
                               'speak_failed':'action_failed'},
                remapping = {'action_in':'action_name',
                             'data_in':'data_name',
                             'num_in':'action_num',
                             'a_num_out':'action_num'})

    asw = ActionServerWrapper(
            'exe_action_plan',
            ExeActionPlanAction,
            wrapped_container = sm_top,
            succeeded_outcomes = ['success'],
            aborted_outcomes = ['action_failed'],
            preempted_outcomes = ['preempted'],
            goal_key = 'goal_message',
            result_key = 'result_message')

    asw.run_server()
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('exe_action_plan', anonymous = True)
    main()
