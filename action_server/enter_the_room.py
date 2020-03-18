#!/usr/bin/env python 
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ドアが開いたことを検出して入室するActionServer
# Author: Issei Iida
# Date: 2019/11/19
# Memo: ドアが閉まっていることを前提条件とする
#--------------------------------------------------------------------

import sys

import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from mimi_common_pkg.msg import EnterTheRoomAction, EnterTheRoomResult

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import speak, BaseCarrier


class EnterTheRoomAS():
    def __init__(self):
        # ActionServer
        self.sas = actionlib.SimpleActionServer('enter_the_room', EnterTheRoomAction,
                                                execute_cb = self.execute,
                                                auto_start = False)
        self.sas.start()
        # Subscriber
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laserCB)
        # Value
        self.bc = BaseCarrier()
        self.result = EnterTheRoomResult()
        self.front_laser_dist = 999.9

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def detection(self, receive_msg):
        target_distance = self.front_laser_dist + receive_msg - 0.05 # -0.05は固定データ
        speak('Please open the door')
        while self.front_laser_dist <= target_distance:
            rospy.sleep(0.1)
        speak('Thank you')
        return target_distance

    def execute(self, goal):
        try:
            rospy.loginfo('Start EnterTheRoom')
            distance = self.detection(goal.distance)
            self.bc.translateDist(distance)
            self.result.data = True
            self.sas.set_succeeded(self.result)
            rospy.loginfo('Finish EnterTheRoom')
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


if __name__ == '__main__':
    rospy.init_node('enter_the_room', anonymous = True)
    ddo_server = EnterTheRoomAS()
    rospy.spin()
