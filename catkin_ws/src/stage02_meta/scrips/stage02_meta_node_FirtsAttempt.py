#!/usr/bin/env python3 

import numpy as np
import ros_numpy
import tf2_ros
import rospy
import time
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist

EQUIPO = "MecaSystem"

def move_base_vel(vx, vy, vw):
    global base_vel_pub

    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw 
    base_vel_pub.publish(twist)

#Suscribers

class Laser():
    u"""Class that handles laser information"""

    def __init__(self):
        # Register the _laser_cb method as a callback to the laser scan topic events
        self._laser_sub = rospy.Subscriber ('/hsrb/base_scan', LaserScan, self._laser_cb)
        self._scan_data = None

    def _laser_cb (self, msg):
        # Laser scan callback function
        self._scan_data = msg

    def get_data(self):
        u"""Function to get the laser value"""
        return self._scan_data    

#Cordenadas 






def main():
    print("Estapa 02 -" + EQUIPO)
    #Inicio del nodo
    rospy.init_node('stage02_meta')   

    #Publishers
    global base_vel_pub
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)

    #Suscribers
    laser = Laser() 
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    def get_coords(tfBuffer):
   
        trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        return trans
    coords = get_coords(tfBuffer)
    print(coords)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
