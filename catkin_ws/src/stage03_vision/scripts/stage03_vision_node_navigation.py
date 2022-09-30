#!/usr/bin/env python3

import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist , PoseStamped
import smach

import ros_numpy
from utils_evasion import *
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler


########## Functions for takeshi states ##########


def get_coords ():
    for i in range(10):   ###TF might be late, try 10 times
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            return trans
        except:
            print ('waiting for tf')
            trans=0
            





def move_base_vel(vx, vy, vw):
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw 
    base_vel_pub.publish(twist)

def move_base(x,y,yaw,timeout=5):
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:  
        move_base_vel(x, y, yaw) 


def move_forward():
    move_base(0.3,0,0,3)
def move_backward():
    move_base(-.3,0,0,3)
def turn_left():
    move_base(0.0,0,0.125*np.pi,2)
def turn_right():
    move_base(0.0,0,-0.125*np.pi,2)

def get_lectura_cuant():
    try:
        lectura=np.asarray(laser.get_data().ranges)
        lectura=np.where(lectura>20,20,lectura) #remove infinito

        right_scan=lectura[60:180]
        left_scan=lectura[540:660]
        front_scan=lectura[345:375]

        sd = np.mean(right_scan)
        si = np.mean(left_scan)
        ft = np.mean(front_scan)
        # if np.mean(left_scan)< 3: si=1
        # if np.mean(right_scan)< 3: sd=1

    except:
        sd,si,ft=0,0    

    return si,sd,ft




##### Define state INITIAL #####

class Inicio (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ','fail']) #shor for success
        
        


    def execute(self,userdata):
        global punto_inicial
    	# Aqui va lo que se desea ejecutar en el estado

        print('inicializando')
        ########
        
        rospy.sleep(1)#### dar tiempo al arbol de tfs de publicarse
        
       
        ##### agregue código para leer la meta en el tópico adecuado
        ###meta_leida=
        
        ############


        ######################################################################
        #meta_leida = PoseStamped() 
         ##### EJEMPLO los equipos deben leer la meta del topico, comentar esta linea
        ####################################################################
        punto_inicial = get_coords()
        print('Tiempo = '+ str(punto_inicial.header.stamp.to_sec()) , punto_inicial.transform )
        print('Meta leida: \n', meta.get_metaPosition())
        print('arrancando')
        return 'succ'

class S1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self,userdata):
    	# Primer estado: girar robot hacia posicion objetivo 
        print('robot Estado S_1')
        #####Accion
        tetha = 0
        meta_x, meta_y = meta.get_metaPosition().pose.position.x,meta.get_metaPosition().pose.position.y
        
        pos_now = get_coords()
        now_x,now_y = pos_now.transform.translation.x, pos_now.transform.translation.y

        x = meta_x - now_x
        y = meta_y - now_y
        
        robot_angle = euler_from_quaternion([0,0,float(pos_now.transform.rotation.z),float(pos_now.transform.rotation.w)])[2]
       
       
        print("x:",x)
        print("y:", y)
        

        if x > 0:
            if y > 0:
                tetha = np.arctan(y/x) - robot_angle
            elif y < 0:
                tetha = np.arctan(y/x) - robot_angle
            elif y < 0.1 and y > -0.1: #pasa a ser condireado como cero
                tetha = 0

            move_base(0,0,tetha,timeout=1)
            print(f"Angulo de movimiento: {tetha}")
        elif x < 0:
            if y > 0:
                tetha = np.pi + (np.arctan(y/x) - robot_angle) 
            elif y < 0:
                tetha = (np.arctan(y/x) - robot_angle) - np.pi
            elif y < 0.1 and y -0.1:
                tetha = np.pi

            move_base(0,0,tetha,timeout=1)
            print(f"Angulo de movimiento: {tetha}")
        elif x < 0.001 and x > -0.001:
            if y > 0:
                tetha = np.pi/2
            elif y < 0:
                tetha = 3*(np.pi/2)

        print("theta: ",tetha)
        time.sleep(3)
        
        return 'outcome1'

class S2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
      

    def execute(self,userdata):
        #Segundo estado checar si hay objeto delante de el 
        print('robot Estado S_2')
        #####Accion
        si,sd,ft=get_lectura_cuant()

        if ft <= 1: #Hay un objeto enfrente (1m)
            print("tf:",ft)
            return 'outcome1'
        else:
            print("Me movi adelante")
            move_forward()
            return 'outcome2'

class S2_5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
      

    def execute(self,userdata):
        #Determina si llega a la meta, si no continua el ciclo
        print('robot Estado S_Intermedio_Final')
        #####Accion
        tetha = 0
        meta_x, meta_y = meta.get_metaPosition().pose.position.x,meta.get_metaPosition().pose.position.y
        pos_now = get_coords()
        now_x = pos_now.transform.translation.x
        now_y = pos_now.transform.translation.y

        x = meta_x - now_x
        y = meta_y - now_y
        if abs(x) < 0.5 and abs(y) < 0.5:
            print(f"LLegaste al destino:{x},{y}")
            punto_final=get_coords()
            print ( 'tiempo = '+ str(punto_final.header.stamp.to_sec()) , punto_final.transform )
            return 'outcome2'
        else:
            return 'outcome1'

class S3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
      

    def execute(self,userdata):
        #Segundo estado checar si hay objetos en la izquierda o derecha
        print('robot Estado S_3')
        #####Accion
        si,sd,ft=get_lectura_cuant()

        if si <= 1: #Hay un objeto a la izquierda(1m)
            print(si)
            if sd <= 1: #Hay un Objeto a la derecha(1m)
                print(sd)
                move_backward()
                return 'outcome2'
            else:
                turn_right()
                return 'outcome1'
        else:
            turn_left()
            return 'outcome1'


class Final(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
      
        


    def execute(self,userdata):
        #Determina si llega a la meta, si no repite el ciclo de nuevo
        print('robot Estado S_Final')
        #####Accion
        tetha = 0
        meta_x, meta_y = meta.get_metaPosition().pose.position.x,meta.get_metaPosition().pose.position.y
        pos_now = get_coords()

        now_x = pos_now.transform.translation.x
        now_y = pos_now.transform.translation.y

        x = meta_x - now_x
        y = meta_y - now_y

        if abs(x) < 0.5 and abs(y) < 0.5:
            print(f"LLegaste al destino:{x},{y}")
            punto_final=get_coords()
            print ( 'tiempo = '+ str(punto_final.header.stamp.to_sec()) , punto_final.transform )
            print(f"Tiempo final: {punto_final.header.stamp.to_sec()-punto_inicial.header.stamp.to_sec()}")
            return 'outcome2'
        else:
            return 'outcome1'







def init(node_name):
    global laser, base_vel_pub, meta, loop
    rospy.init_node(node_name)
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    laser = Laser()  
    meta = metaPoint()
    loop = rospy.Rate(10)


#Entry point
if __name__== '__main__':

    print("STATE MACHINE...")
    init("stage02_meta")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    with sm:
        #State machine for evasion
        smach.StateMachine.add("INICIO",   Inicio(),  transitions = {'fail':'END', 'succ':'s_1'})
        smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_2','outcome2':'END'})
        #smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'END','outcome2':'END'})
        smach.StateMachine.add("s_2",   S2(),  transitions = {'outcome1':'s_3','outcome2':'s_2_5'})
        smach.StateMachine.add("s_2_5",   S2_5(),  transitions = {'outcome1':'s_1','outcome2':'END'})
        smach.StateMachine.add("s_3",   S3(),  transitions = {'outcome1':'FINAL','outcome2':'s_3'})
        smach.StateMachine.add("FINAL",   Final(),  transitions = {'outcome1':'s_2','outcome2':'END'})
        


outcome = sm.execute()