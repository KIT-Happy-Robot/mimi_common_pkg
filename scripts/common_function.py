#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: 使用頻度が高い処理を纏めたPythonスクリプト
# Author: Issei Iida
# Date: 2019/09/07
# Memo:
#--------------------------------------------------------------------

# Python
import time
from yaml import load
import sys
# ROS
import rospy
import rosparam
from std_msgs.msg import String, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gcp_texttospeech.srv import TTS

# Grobal
pub_speak = rospy.Publisher('/tts', String, queue_size = 1)
pub_m6 = rospy.Publisher('/m6_controller/command', Float64, queue_size = 1)
tts_srv = rospy.ServiceProxy('/tts', TTS)

# 話す
def speak(phrase):
    tts_srv(phrase)


# m6(首のサーボモータ)の制御
def m6Control(value):
    data = Float64()
    data = value
    rospy.sleep(0.1)
    pub_m6.publish(data)


# 足回りの制御
class BaseCarrier():
    def __init__(self):
        # Publisher
        self.pub_twist = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        self.twist_value = Twist()

    # 指定した距離を並進移動
    def translateDist(self, distance):
        try:
            target_time = abs(distance / 0.2)
            if distance >0:
                self.twist_value.linear.x = 0.24
            elif distance < 0:
                self.twist_value.linear.x = -0.24
            rate = rospy.Rate(30)
            start_time = time.time()
            end_time = time.time()
            while end_time - start_time <= target_time:
                self.pub_twist.publish(self.twist_value)
                end_time = time.time()
                rate.sleep()
            self.twist_value.linear.x = 0.0
            self.pub_twist.publish(self.twist_value)
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass

    # 指定した角度(±360度の範囲)だけ回転
    def angleRotation(self, degree):
        try:
            while degree > 180:
                degree = degree - 90
            while degree < -180:
                degree = degree + 90
            angular_speed = 90.0 #[deg/s]
            target_time = abs(1.76899*(degree /angular_speed))  #[s]
            if degree >= 0:
                self.twist_value.angular.z = (angular_speed * 3.14159263 / 180.0) #rad
            elif degree < 0:
                self.twist_value.angular.z = -(angular_speed * 3.14159263 / 180.0) #rad
            rate = rospy.Rate(500)
            start_time = time.time()
            end_time = time.time()
            while end_time - start_time <= target_time:
                self.pub_twist.publish(self.twist_value)
                end_time = time.time()
                rate.sleep()
            self.twist_value.angular.z = 0.0
            self.pub_twist.publish(self.twist_value)
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


# 文字列をパラメータの/location_dictから検索して位置座標を返す
def searchLocationName(target_param, target_name):
    location_dict = rosparam.get_param(target_param)
    if target_name in location_dict:
        print location_dict[target_name]
        return location_dict[target_name]
    else:
        return 'faild'
