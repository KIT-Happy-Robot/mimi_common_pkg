#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: 人接近を行うActionServer
# Author: Issei Iida
# Date: 2019/10/13
# Memo: 台風19号ハギビス
#---------------------------------------------------------------------

#Python関系
import sys
import time
#ROS関系
import rospy
import rosparam
from actionlib import *
from mimi_common_pkg.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from get_distance_pcl.msg import Coordinate_xyz
from smach_ros import ActionServerWrapper
from smach import StateMachine
import smach_ros
import smach

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_action_client import *
from common_function import *


class Localization(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['localize',
                            'not_localize'],
                input_keys = ['goal_in'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state LOCALIZATION')
            result = localizePersonAC()
            result = 'success'
            if result == 'success':
                speak('I found person')
                rospy.loginfo('Localization Succes')
                return 'localize'
            else:
                speak('I can`t find person')
                rospy.loginfo('Localization Failed')
                return 'not_localize'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class GetCootdinate(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['get',
                            'not_get'],
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

    def orientationCB(self, receive_msg):
        self.person_coord_z = receive_msg.pose.pose.orientation.z
        self.person_coord_w = receive_msg.pose.pose.orientation.w

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state GET_COORDINATE')
            self.pub_coord_req.publish(True)
            while not rospy.is_shutdown() and self.person_coord_x == 0.00:
                rospy.loginfo('Waiting for coordinate')
                rospy.sleep(1.0)
            rospy.loginfo('Got Coordinate')
            self.coord_list.append(self.person_coord_x)
            self.coord_list.append(self.person_coord_y)
            self.coord_list.append(self.person_coord_z)
            self.coord_list.append(self.person_coord_w)
            print self.coord_list
            userdata.coord_out = self.coord_list
            return 'get'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['arrive',
                            'not_arrive'],
                input_keys = ['result_message',
                              'coord_in'],
                output_keys = ['result_message'])

        self.result = 'null'

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state NAVIGATION')
            ap_result = userdata.result_message
            coord_list = userdata.coord_in
            m6Control(0.3)
            rosparam.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', str(0.5))
            print rosparam.get_param('/move_base/DWAPlannerROS/xy_goal_tolerance')
            rospy.sleep(0.1)
            result = navigationAC(coord_list)
            print result
            if self.result == True:
                m6Control(0.4)
                speak('I came close to person')
                #ap_result = result
                #userdata.result_message.data = ap_result
                userdata.result_message.data = self.result
                return 'arrive'
            else:
                speak('I can`t came close to person')
                #ap_result = result
                #userdata.result_message.data = ap_result
                userdata.result_message.data = self.result
                return 'not_arrive'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


def main():
    sm_top = StateMachine(
            outcomes = ['success',
                        'localize_failed',
                        'navi_failed',
                        'get_failed',
                        'preempted'],
            input_keys = ['goal_message',
                          'result_message'],
            output_keys = ['result_message'])

    with sm_top:
        StateMachine.add(
                'LOCALIZATION',
                Localization(),
                transitions = {'localize':'GET_COORDINATE',
                               'not_localize':'localize_failed'},
                remapping = {'goal_in':'goal_message'})

        StateMachine.add(
                'GET_COORDINATE',
                GetCootdinate(),
                transitions = {'get':'NAVIGATION',
                               'not_get':'get_failed'},
                remapping = {'coord_out':'person_coord'})

        StateMachine.add(
                'NAVIGATION',
                Navigation(),
                transitions = {'arrive':'success',
                               'not_arrive':'navi_failed'},
                remapping = {'result_message':'result_message',
                             'coord_in':'person_coord'})

    asw = ActionServerWrapper(
            'approach_person',
            ApproachPersonAction,
            wrapped_container = sm_top,
            succeeded_outcomes = ['success'],
            aborted_outcomes = ['find_failed',
                                'get_failed',
                                'navi_failed'],
            preempted_outcomes = ['preempted'],
            goal_key = 'goal_message',
            result_key = 'result_message')

    asw.run_server()
    rospy.spin()
    

if __name__ == '__main__':
    rospy.init_node('approach_person', anonymous = True)
    main()
