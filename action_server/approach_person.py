#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: 人接近を行うActionServer
# Author: Issei Iida
# Date: 2019/10/13
# Memo: 台風19号ハギビス
#---------------------------------------------------------------------

PACKAGE = 'base_local_planner'
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
import roslib;roslib.load_manifest(PACKAGE)
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
            rospy.sleep(1.0)
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

        self.person_coord_x = 0.00
        self.person_coord_y = 0.00
        self.person_coord_z = 0.00
        self.person_coord_w = 0.00
        self.coord_list = []

    def personCoordCB(self, receive_msg):
        self.person_coord_x = receive_msg.world_x
        self.person_coord_y = receive_msg.world_y
        # self.person_coord_z = receive_msg.world_z
        # self.person_coord_w = receive_msg.world_w

    def orientationCB(self, receive_msg):
        self.person_coord_z = receive_msg.pose.pose.orientation.z
        self.person_coord_w = receive_msg.pose.pose.orientation.w

    def execute(self, userdata):
        rospy.loginfo('Executing state GET_COORDINATE')
        rospy.sleep(0.1)
        self.pub_coord_req.publish(True)
        while not rospy.is_shutdown() and self.person_coord_x == 0.00:
            rospy.sleep(0.1)
        self.coord_list.append(self.person_coord_x)
        self.coord_list.append(self.person_coord_y)
        self.coord_list.append(self.person_coord_z)
        self.coord_list.append(self.person_coord_w)
        print self.coord_list
        userdata.coord_out = self.coord_list
        self.person_coord_x = 0.00
        self.person_coord_y = 0.00
        self.person_coord_z = 0.00
        self.person_coord_w = 0.00
        self.coord_list = []
        return 'get_success'


class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['navi_success', 'navi_failure'],
                             input_keys = ['result_message', 'coord_in'],
                             output_keys = ['result_message'])
        # self.xy_goal_tolerance_pub = rospy.Publisher('/move_base/DWAPlannerROS/parameter_update',
        #                                              Config,
        #                                              queue_size = 1)
        self.client = dynamic_reconfigure.client.Client('move_base/TrajectoryPlannerROS', timeout=4,config_callback=self.execute)

    def execute(self, userdata):
        rospy.loginfo('Executing state NAVIGATION')
        coord_list = userdata.coord_in
        rospy.sleep(0.5)
        r = rospy.Rate(0.3)
        while not rospy.is_shutdown():
            client.update_configuration({"xy_goal_tolerance":0.8})
            r.sleep()
        result = navigationAC(coord_list)
        self.coord_list = []
        if result == True:
            m6Control(0.2)
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
