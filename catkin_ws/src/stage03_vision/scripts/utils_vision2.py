import rospy
import time
import numpy as np
import tf
import moveit_commander
import moveit_msgs.msg
import cv2
from geometry_msgs.msg import Twist

rospy.init_node('base_and_sensor')    ### Conectamos/creamos un nodo llamado base and sensor 
base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)## Declaramos un publisher que ha 
###de enviar mensajes tipo Twist al topico hsrb/command_velocity
listener = tf.TransformListener()
aux2=0

def move_base_vel(vx, vy, vw):
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw 
    base_vel_pub.publish(twist)

def move_base(x,y,yaw,timeout=5):
    rospy.init_node('base_and_sensor')    ### Conectamos/creamos un nodo llamado base and sensor 
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)## Declaramos un publisher que ha 
    ###de enviar mensajes tipo Twist al topico hsrb/command_velocity
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:  
        move_base_vel(x, y, yaw)

def move_h_g():
    head = moveit_commander.MoveGroupCommander('head')
    whole_body=moveit_commander.MoveGroupCommander('whole_body_weighted')
    arm =  moveit_commander.MoveGroupCommander('arm')

    arm.set_named_target('go')
    arm.go()

    head.go(np.array((0,-.15*np.pi)))

def centroids(contours, points, coord):
    for contour in contours:
        xyz=[]
        M = cv2.moments(contour) #MOMENTOS ESADISTICOS DE LA IMAGEN
    
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        boundRect = cv2.boundingRect(contour)

        for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
            for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                aux=(np.asarray((points['x'][ix,jy],points['y'][ix,jy],points['z'][ix,jy])))
                if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                    'reject point'
                else:
                    xyz.append(aux)
                
        xyz=np.asarray(xyz)
        cent=xyz.mean(axis=0)
        #print(cent)
        #aux = np.asarray(cent)
        coord.append(cent)
    return coord

def esc_coord(aux):
    temp = []
    coord_rect = {}
    global aux2

    try:
        f = open("/home/cire2022/CIRE2022/catkin_ws/src/stage03_vision/scripts/prueba.txt", "x")
        f.close()
        f = open("/home/cire2022/CIRE2022/catkin_ws/src/stage03_vision/scripts/prueba.txt", "w")
        f.write("Coordenadas:\n")
        f.close()
    except:
        print("File has already been created")
    
    for i in range(aux):
        coordnd, ns = listener.lookupTransform('map','Object'+str(i),rospy.Time(0))
        coord_rect['Object'+str(i) + ":"] = str(coordnd)
    
    f = open("/home/cire2022/CIRE2022/catkin_ws/src/stage03_vision/scripts/prueba.txt", "r")
    for line in f:
        temp.append(f.read(8))
    f.close()

    f = open("/home/cire2022/CIRE2022/catkin_ws/src/stage03_vision/scripts/prueba.txt", "a")
    for cr in coord_rect: 
        if cr in temp:
            print("Object has already been registered")
        else:
            f.write(str(cr)+str(coord_rect[cr]) + "\n")
        aux2+=1
    f.flush()
    f.close()