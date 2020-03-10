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
from common_function import m6Control


class Detection():
    def __init__(self):
        # Publisher
        self.pub_twist = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        # Subscriber
        self.sub_recog = rospy.Subscriber('/recog_obj', String, self.recogCB)
        # Service
        self.obj_recog = rospy.ServiceProxy('/object/recognize', RecognizeExistence)
        # Value
        self.twist_value = Twist()
        self.timeout = 0

    def recogCB(self, receive_msg):
        obj_list = receive_msg.data.split(" ")
        for i in range(len(obj_list)):
            if obj_list[i] == 'person':
                self.person_flg = True

    def execute(self, receive_msg):
        person_flg = False
        self.timeout = time.time() + 30
        # self.data.target = receive_msg
        result = self.obj_recog(self.data.target)
        self.twist_value.angular.z = 0.3
        while not rospy.is_shutdown() and person_flg == False:
            # result = self.obj_recog(self.data.target)
            if time.time() > self.timeout:
                self.twist_value.angular.z == 0
                self.pub_twist.publish(self.twist_value)
                result = False
                break
            else:
                self.pub_twist.publish(self.twist_value)
                rospy.sleep(0.5)
            print person_flg
        self.twist_value.angular.z == 0
        self.pub_twist.publish(self.twist_value)
        return True




class LocalizeObjectAS():
    def __init__(self):
        # ActionServer
        self.sas = actionlib.SimpleActionServer(
                'localize_object',
                LocalizeObjectAction,
                execute_cb = self.execute,
                auto_start = False)
        # Value
        self.result = LocalizeObjectResult()
        self.data = RecognizeExistence()
        self.detection = Detection()

    def execute(self, goal):
        self.sas.start()
        m6Control(-0.1)
        result = self.detection.execute(goal.data)
        m6Control(0.3)
        self.result.data = result
        self.sas.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node('localize_object', anonymous = True)
    fp_server = LocalizeObjectAS()
    rospy.spin()
