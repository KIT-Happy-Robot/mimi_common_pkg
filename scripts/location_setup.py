#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
# Title: Locationの名前と座標を登録するサービスサーバー
# Author: Issei Iida
# Date: 2020/02/04
#-----------------------------------------------------------

import rospy
import rosparam
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from mimi_common_pkg.srv import LocationSetup, LocationSetupResponse


class AddLocation():
    def __init__(self):
        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.getOdomCB)
        # Value
        self.location_dict = {}
        self.location_pose_x = 0.00
        self.location_pose_y = 0.00
        self.location_pose_z = 0.00
        self.location_pose_w = 0.00

    def getOdomCB(self, receive_msg):
        if receive_msg.child_frame_id == 'base_footprint':
            self.location_pose_x = receive_msg.pose.pose.position.x
            self.location_pose_y = receive_msg.pose.pose.position.y
            self.location_pose_z = receive_msg.pose.pose.orientation.z
            self.location_pose_w = receive_msg.pose.pose.orientation.w

    def getMapCoordinateCB(self, receive_msg):
        self.location_pose_x = receive_msg.x
        self.location_pose_y = receive_msg.y

    def addLocation(self, name):
        if name in self.location_dict:
            rospy.loginfo('*LocationName already exists*')
            return False
        else:
            rospy.loginfo('Add <' + name + '> position')
            self.location_dict[name] = []
            self.location_dict[name].append(self.location_pose_x)
            self.location_dict[name].append(self.location_pose_y)
            self.location_dict[name].append(self.location_pose_z)
            self.location_dict[name].append(self.location_pose_w)
            print self.location_dict[name]
            return True

    def saveLocation(self, param_name):
        rospy.set_param('location_dict', self.location_dict)
        rosparam.dump_params('/home/athome/catkin_ws/src/mimi_common_pkg/config/'
                             + param_name + '.yaml', '/location_dict')
        print rosparam.get_param('location_dict')
        rospy.loginfo('Saving complete')
        return True


class SetupServer():
    def __init__(self):
        self.server = rospy.Service('/location_setup', LocationSetup, self.execute)
        self.al = AddLocation()

    def execute(self, req):
        rospy.loginfo('Start location_setup')
        state = req.state
        name = req.name
        if state == 'add':
            rospy.loginfo('Add location')
            return LocationSetupResponse(result = self.al.addLocation(name))
        elif state == 'save':
            rospy.loginfo('Save location')
            return LocationSetupResponse(result = self.al.saveLocation(name))
        else:
            rospy.loginfo('What are you doing?')
            return LocationSetupResponse(result = False)


if __name__ == '__main__':
    rospy.init_node('location_setup', anonymous = True)
    ss = SetupServer()
    rospy.spin()
