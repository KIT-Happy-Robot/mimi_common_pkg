#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: オブジェクトの方向を向くためのActionServer
# Author: Issei Iida
# Date: 2020/01/15
# Memo:
#---------------------------------------------------------------------

# Pyhon
import sys
import time
# ROS
import rospy
from std_msgs.msg import String
from mimi_common_pkg.msg import LocalizeObjectAction, LocalizeObjectResult
from mimi_common_pkg.srv import RecognizeExistence
from geometry_msgs.msg import Twist
import actionlib

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import BaseCarrier, m6Control


class LocalizeObjectAS():
    def __init__(self):
        # ActionServer
        self.sas = actionlib.SimpleActionServer(
                'localize_object',
                LocalizeObjectAction,
                execute_cb = self.execute,
                auto_start = False)
        # Publisher
        self.pub_twist = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        # Subscriber
        self.sub_recog = rospy.Subscriber('/recog_obj', String, self.recogCB)
        # Service
        self.obj_recog = rospy.ServiceProxy('/object/recognize', RecognizeExistence)
        
        self.bc = BaseCarrier()
        self.result = LocalizeObjectResult()
        self.data = RecognizeExistence()
        self.person_flg = False
        self.timeout = 0
        self.twist_value = Twist()

        self.sas.start()

    def recogCB(self, receive_msg):
        obj_list = receive_msg.data.split(" ")
        for i in range(len(obj_list)):
            if obj_list[i] == 'person':
                self.person_flg = True

    def detection(self, receive_msg):
        self.person_flg = False
        self.timeout = time.time() + 30
        # self.data.target = receive_msg
        result = self.obj_recog(self.data.target)
        self.twist_value.angular.z = 0.3
        while not rospy.is_shutdown() and result.existence == False:

        # while not rospy.is_shutdown() and not self.person_flg == True:
            self.pub_twist.publish(self.twist_value)
            rospy.sleep(0.3)
            print self.person_flg
            # result = self.obj_recog(self.data.target)
            # if time.time() > self.timeout:
            #     return False
        self.person_flg = False
        return True

    def execute(self, goal):
        rospy.loginfo('Start LocalizeObject')
        m6Control(-0.1)
        rospy.loginfo('Start detection')
        result = self.detection(goal.data)
        self.twist_value.angular.z = 0.0
        self.pub_twist.publish(self.twist_value)
        rospy.loginfo('Finish detection')
        m6Control(0.3)
        if result is True:
            self.result.data = result
            self.sas.set_succeeded(self.result)
        else:
            result.data = result
            self.sas.set_succeeded(self.result)
        rospy.loginfo('Finish LocalizeObject')


if __name__ == '__main__':
    rospy.init_node('localize_object', anonymous = True)
    fp_server = LocalizeObjectAS()
    rospy.spin()
