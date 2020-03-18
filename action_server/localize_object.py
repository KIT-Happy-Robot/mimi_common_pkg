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
from mimi_common_pkg.srv import RecognizeCount
from geometry_msgs.msg import Twist
import actionlib

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import m6Control


class Detection():
    def __init__(self):
        # Publisher
        self.pub_twist = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        # Service
        self.obj_recog = rospy.ServiceProxy('/object/recognize', RecognizeCount)
        # Value
        self.twist_value = Twist()
        self.data = RecognizeCount()
        self.timeout = 0

    def execute(self, receive_msg):
        person_flg = False
        self.data.target = receive_msg
        self.timeout = time.time() + 30
        self.twist_value.angular.z = 0.3
        while not rospy.is_shutdown() and person_flg == False:
            person_flg = self.obj_recog(self.data.target).num
            if time.time() > self.timeout:
                self.twist_value.angular.z == 0
                self.pub_twist.publish(self.twist_value)
                return False
            else:
                self.pub_twist.publish(self.twist_value)
                rospy.sleep(0.5)
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
        self.sas.start()
        # Value
        self.result = LocalizeObjectResult()
        self.detection = Detection()

    def execute(self, goal):
        m6Control(-0.4)
        rospy.sleep(1.0)
        result = self.detection.execute(goal.data)
        self.result.data = result
        self.sas.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node('localize_object', anonymous = True)
    fp_server = LocalizeObjectAS()
    rospy.spin()
