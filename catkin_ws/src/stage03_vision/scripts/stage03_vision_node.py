#!/usr/bin/env python3
# license removed for brevity
import rospy
import time
import numpy as np
#%matplotlib inline
import matplotlib.pyplot as plt
import ros_numpy
import time
import tf
import moveit_commander
import moveit_msgs.msg
from gazebo_ros import gazebo_interface
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Quaternion ,TransformStamped, Twist
import sys
from utils_vision2 import *
import cv2
import os
from cv_bridge import CvBridge

#base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)## Declaramos un publisher que ha 
##de enviar mensajes tipo Twist al topico hsrb/command_velocity
broadcaster= tf.TransformBroadcaster()
aux1 = 0


def callback_pcv(msg):
    global aux1, aux
    aux=aux1
    print("Point cloud received: " + str(msg.width) + "x" + str(msg.height))
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    rgb = arr['rgb'].copy().view(np.uint32)
    r,g,b = ((rgb >> 16) & 255), ((rgb >> 8) & 255), (rgb & 255)
    img = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
    im_hsv = cv2.cvtColor(np.float32(img), cv2.COLOR_BGR2HSV)
    h_min=0
    h_max=145
    region = (im_hsv > h_min) & (im_hsv < h_max)
    idx,idy=np.where(region[:,:,0] )
    mask= np.zeros((480,640))
    mask[idx,idy]=255

    kernel = np.ones((5, 5), np.uint8)
    eroded_mask=cv2.erode(mask,kernel)
    eroded_mask=cv2.erode(eroded_mask,kernel)
    eroded_mask=cv2.erode(eroded_mask, kernel)
    dilated_mask=cv2.dilate(eroded_mask,kernel)

    contours, hierarchy = cv2.findContours(dilated_mask.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    coord1 = []
    coord2 = centroids(contours, arr, coord1)

    for c in coord2:
        x,y,z=c
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            broadcaster.sendTransform((x,y,z),(0,0,0,1), rospy.Time.now(), 'Object'+str(aux),"head_rgbd_sensor_link")
            print(aux)
            aux += 1
    


def main():
    global aux, aux1
    #move_base(0,0,0.5*np.pi)
    #move_h_g()    
    rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, callback_pcv)
    rospy.sleep(3)
    #rospy.Subscriber('hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, callback_image)
    #rospy.spin()
    esc_coord(aux)
    aux1 = aux
    print(aux1)

if __name__ == '__main__':
    main()




