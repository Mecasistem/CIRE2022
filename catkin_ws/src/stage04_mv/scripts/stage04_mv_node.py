#!/usr/bin/env python3

import rospy
import time
import math
import numpy as np
import ros_numpy
import roslib
import actionlib
from rosgraph_msgs.msg import Log
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction,MoveBaseGoal
import os

class LogRos():
    u"""Class that handles logs information"""

    def __init__(self, node_search ,msg_search):
        # Register the _laser_cb method as a callback to the laser scan topic events
        self._laser_sub = rospy.Subscriber ('/rosout_agg',
                                           Log, self._log_cb)
        self._scan_logs = None
        self._search = msg_search
        self._node_search = node_search
        self._isFound = False

    def _log_cb (self, msg):
        if  msg.name in self._node_search:
            if msg.msg == self._search:
                self._isFound = True
        self._scan_data = msg

    def get_data(self):
        u"""Function to get the log value"""
        return self._scan_data

    def log_is_found(self):
        u"""Funtion to ask if particully log is faound"""
        return self._isFound

def main():
    rospy.init_node('stagedebug_04') 
    rate =rospy.Rate(2) 
    loglistener = LogRos("/explore","Exploration stopped.")
    client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    client.wait_for_server()

    #Posicion incial 
    posinicial = MoveBaseGoal()
    posinicial.target_pose.header.frame_id = "map"
    posinicial.target_pose.pose.orientation.w = 1.0
    posinicial.target_pose.pose.position.x = 0.0
    posinicial.target_pose.pose.position.y = 0.0
    posinicial.target_pose.pose.position.z = 0.0

    while not rospy.is_shutdown():
        if loglistener.log_is_found():
            print(loglistener.log_is_found())
            print("Exploracion terminada")
            client.send_goal(posinicial)
            client.wait_for_result(rospy.Duration.from_sec(5.0))
            os.chdir("/home/cire2022/CIRE2022/catkin_ws/src/stage04_mv")
            os.system("mkdir maps")
            os.chdir("/home/cire2022/CIRE2022/catkin_ws/src/stage04_mv/maps")
            os.system("rosrun map_server map_saver -f mymap map:=/map")
            print("Mapas Guardados")
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass