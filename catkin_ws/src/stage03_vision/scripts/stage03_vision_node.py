#!/usr/bin/env python3
# license removed for brevity
import rospy
import time
import numpy as np
#%matplotlib inline
import matplotlib.pyplot as plt
import numpy as np
import ros_numpy
import time
import tf
import moveit_commander
import moveit_msgs.msg
from gazebo_ros import gazebo_interface
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Pose, Quaternion ,TransformStamped
from geometry_msgs.msg import Twist
import sys
from utils_notebooks import *
import cv2
import os
from geometry_msgs.msg import Twist

#base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)## Declaramos un publisher que ha 
###de enviar mensajes tipo Twist al topico hsrb/command_velocity

rospy.init_node("recognition")

#def move_base_vel(vx, vy, vw):
#    twist = Twist()
#    twist.linear.x = vx
#    twist.linear.y = vy
#    twist.angular.z = vw 
#    base_vel_pub.publish(twist)
    
#move_base_vel(0 , 0, 3*np.pi)

#listener = tf.TransformListener()
#broadcaster= tf.TransformBroadcaster()

#head = moveit_commander.MoveGroupCommander('head')
#whole_body=moveit_commander.MoveGroupCommander('whole_body_weighted')
#arm =  moveit_commander.MoveGroupCommander('arm')

#arm.set_named_target('go')
#arm.go()

#head.go(np.array((0,-.15*np.pi)))


rgbd = RGBD()

image=rgbd.get_image()  #dimensiones de la imagen
image.shape()    # una matriz (arreglo tipo numpy) 480px por 680 px 3 canales
points= rgbd.get_points() 

image.dtype()
plt.imshow(image)



