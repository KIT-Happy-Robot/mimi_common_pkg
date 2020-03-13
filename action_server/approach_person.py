#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: 人接近を行うActionServer
# Author: Issei Iida
# Date: 2019/10/13
# Memo: 台風19号ハギビス
#---------------------------------------------------------------------

# Python
import sys
import time
# ROS
import rospy
import rosparam
import actionlib
from mimi_common_pkg.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from get_distance_pcl.msg import Coordinate_xyz
from smach_ros import ActionServerWrapper
from smach import StateMachine
import smach_ros
import smach
import dynamic_reconfigure.client


sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *


class Localize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['localize_success', 'localize_failure'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LOCALIZATION')
        result = localizeObjectAC('person')
        if result == True:
            # speak('I found person')
            return 'localize_success'
        else:
            # speak('I can`t find person')
            return 'localize_failure'


class GetCootdinate(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['get_success', 'get_failure'],
                             output_keys = ['coord_out'])
        # Publisher
        self.pub_coord_req = rospy.Publisher('/move_close_human/human_detect_flag',
                                             Bool,
                                             queue_size = 1) 
        # Subscriber
        self.sub_pcl = rospy.Subscriber('/get_distance_pcl/Coordinate_xyz',
                                        Coordinate_xyz,
                                        self.personCoordCB)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.orientationCB)
        # Value
        self.p_coord_list = []

    def personCoordCB(self, receive_msg):
        self.p_coord_list[0] = receive_msg.world_x
        self.p_coord_list[1] = receive_msg.world_y

    def orientationCB(self, receive_msg):
        self.p_coord_list[2] = receive_msg.pose.pose.orientation.z
        self.p_coord_list[3] = receive_msg.pose.pose.orientation.w

    def execute(self, userdata):
        rospy.loginfo('Executing state GET_COORDINATE')
        rospy.sleep(0.1)
        self.pub_coord_req.publish(True)
        # while not rospy.is_shutdown() and self.person_coord_x == 0.00:
        while not rospy.is_shutdown() and len(self.p_coord_list) <= 4:
            rospy.sleep(0.1)
        userdata.coord_out = self.p_coord_list
        self.p_coord_list = []
        return 'get_success'


class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['navi_success', 'navi_failure'],
                             input_keys = ['result_message', 'coord_in'],
                             output_keys = ['result_message'])
        # Dynparam
        self.dwa_c = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
        self.realsense_c = dynamic_reconfigure.client.Client('/move_base/local_costmap/realsense_layer')

    def setDynparam(self, state):
        if state == 'set':
            goal_tolerance_p = {'xy_goal_tolerance':0.8, 'yaw_goal_tolerance':6.2}
            realsense_p = {'enabled':False}
        elif state == 'defalt':
            goal_tolerance_p = {'xy_goal_tolerance':0.15, 'yaw_goal_tolerance':0.08}
            realsense_p = {'enabled':True}
        self.dwa_c.update_configuration(goal_tolerance_p)
        self.realsense_c.update_configuration(realsense_p)
        rospy.sleep(1.0)
 
    def execute(self, userdata):
        rospy.loginfo('Executing state NAVIGATION')
        coord_list = userdata.coord_in
        self.setDynparam('set')
        result = navigationAC(coord_list)
        self.setDynparam('defalt')
        m6Control(0.3)
        if result == True:
            # speak('I came close to person')
            userdata.result_message.data = result
            return 'navi_success'
        else:
            # speak('I can`t came close to person')
            userdata.result_message.data = result
            return 'navi_failure'


def main():
    sm_top = StateMachine(
            outcomes = ['success',
                        'action_failed',
                        'preempted'],
            input_keys = ['goal_message',
                          'result_message'],
            output_keys = ['result_message'])

    with sm_top:
        StateMachine.add(
                'LOCALIZE',
                Localize(),
                transitions = {'localize_success':'GET_COORDINATE',
                               'localize_failure':'action_failed'})

        StateMachine.add(
                'GET_COORDINATE',
                GetCootdinate(),
                transitions = {'get_success':'NAVIGATION',
                               'get_failure':'action_failed'},
                remapping = {'coord_out':'person_coord'})

        StateMachine.add(
                'NAVIGATION',
                Navigation(),
                transitions = {'navi_success':'success',
                               'navi_failure':'action_failed'},
                remapping = {'result_message':'result_message',
                             'coord_in':'person_coord'})

    asw = ActionServerWrapper(
            'approach_person',
            ApproachPersonAction,
            wrapped_container = sm_top,
            succeeded_outcomes = ['success'],
            aborted_outcomes = ['action_failed'],
            preempted_outcomes = ['preempted'],
            goal_key = 'goal_message',
            result_key = 'result_message')

    asw.run_server()
    rospy.spin()
    

if __name__ == '__main__':
    rospy.init_node('approach_person', anonymous = True)
    main()
