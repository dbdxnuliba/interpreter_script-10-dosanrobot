#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
import sys
import numpy as np
from sensor_msgs.msg import Joy
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_ROBOT.py 

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
m_joyAnalogFlag = False
m_xyCompareFlag = False
m_joyButtonFlag = False
m_joyJogFlag = 0
m_joyJogVel = 0.0
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *
from markers import *

def shutdown():
    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

""" Procesa los valores que llegan del nodo joy, y toma las decisiones de movimiento y control del gripper """
def joy_cb(msg):
    global m_joyAnalogFlag
    global m_xyCompareFlag
    global m_joyJogFlag
    global m_joyJogVel
    global moveObject

    initialPosition = [0, 0, 90, 0, -90, 0]
    homePosition = [0, 0, 0, 0, 0, 0]

    if msg.buttons[7] == 1 and msg.buttons[6] == 1:
        r.movej(initialPosition, 50, 50) #posición inicial para movimiento del robot
    elif msg.buttons[10] == 1:
        r.movej(homePosition, 50, 50) #posición de no trabajo del robot
    
    if msg.buttons[0] == 1:
        srv_robotiq_2f_move(0.4) #Recoger objeto corresponde al radio del objeto (X)
        moveObject = True

    if msg.buttons[2] == 1:
        srv_robotiq_2f_move(0) #abrir pinza (TRIANGULO)
        moveObject = False
    
    if msg.buttons[1] == 1 and not moveObject:
        srv_robotiq_2f_move(0.8) #cerrar pinza (O)
    
    #Activación de banderas según el eje del movimiento, depende de los joystick del control
    if msg.axes[4] != 0 or msg.axes[0] != 0 or msg.axes[1] != 0:
        m_joyAnalogFlag = True
    else:
        m_joyAnalogFlag = False

    if msg.axes[1] != 0 or msg.axes[0] != 0:
        if abs(msg.axes[1]) > abs(msg.axes[0]):
            m_xyCompareFlag = False
        else:
            m_xyCompareFlag = True
    
    # se asignan velocidades y dirección del movimiento
    if m_joyAnalogFlag and m_joyJogFlag == -1:
        if msg.axes[4] > 0: #z+
            m_joyJogFlag = JOG_AXIS_TASK_Z 
            m_joyJogVel = 20
        if msg.axes[4] < 0: #z-
            m_joyJogFlag = JOG_AXIS_TASK_Z 
            m_joyJogVel = -20
        if msg.axes[1] > 0 and m_xyCompareFlag == 0: #x-
            m_joyJogFlag = JOG_AXIS_TASK_X 
            m_joyJogVel = -20
        if msg.axes[1] < 0 and m_xyCompareFlag == 0: #x+
            m_joyJogFlag = JOG_AXIS_TASK_X
            m_joyJogVel = 20
        if msg.axes[0] > 0 and m_xyCompareFlag == 1: #y-
            m_joyJogFlag = JOG_AXIS_TASK_Y
            m_joyJogVel = -20
        if msg.axes[0] < 0 and m_xyCompareFlag == 1: #y+
            m_joyJogFlag = JOG_AXIS_TASK_Y
            m_joyJogVel = 20
        r.jog(m_joyJogFlag, COORDINATE_SYSTEM_TOOL, m_joyJogVel)
    else: #Condición cuando se presiona algún botón o se mueve un joystick en alguna dirección no especificada
        if not m_joyAnalogFlag:
            r.jog(m_joyJogFlag, COORDINATE_SYSTEM_TOOL, 0)
            m_joyJogFlag = -1

def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.
    """
    cti = np.cos(theta)
    cai = np.cos(alpha)
    sti = np.sin(theta)
    sai = np.sin(alpha)

    T = np.array([[cti, -cai*sti, sai*sti, a*cti], [sti, cai*cti, -sai*cti, a*sti], [0, sai, cai, d], [0, 0, 0, 1]])
    return T
    
    

def fkine_ur5(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6
    """
    ### Recibe q en arreglo tipo columna de 6x1
    # Matrices DH (completar)
    T1 = dh(0.1525, q[0], 0, -pi/2)
    T2 = dh(0, q[1]-pi/2, 0.62, 0)
    T3 = dh(0.0345, q[2]+pi/2, 0, pi/2)
    T4 = dh(0.559, q[3]+pi, 0, pi/2)
    T5 = dh(0, q[4]+pi, 0, pi/2)
    T6 = dh(0.121, q[5], 0, 0)
    
    # Efector final con respecto a la base
    T = np.dot(np.dot(np.dot(np.dot(np.dot(T1, T2), T3), T4), T5), T6)
    return T

def meters2milimiters(pos):
    i=0
    posMilimiter = [0,0,0]
    for r in pos:
        posMilimiter[i] = r / 1000
        if i==2:
            posMilimiter[i] = posMilimiter[i] + 0.15
            break
        i = i + 1
    return posMilimiter

r = CDsrRobot(ROBOT_ID, ROBOT_MODEL)
pi=np.pi
moveObject = False
bmarker_des = BallMarker(color['BLUE']) # Color AZUL de la esfera como objecto a manipular
pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10) #Detener la comunicación con el robot
sub_joy  = rospy.Subscriber("joy", Joy, joy_cb) #Captuar cuando el control de Ps4 es manipulado
srv_robotiq_2f_move = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/robotiq_2f_move', Robotiq2FMove) #manipular la pinza
if __name__ == "__main__":
    rospy.init_node('joy_ps4_py') 
    rospy.on_shutdown(shutdown)

    srv_robotiq_2f_move(0.8) #cerrar pinza
    r.movej([0, 0, 0, 0, 0, 0], 50, 50) #llevar el robot a la posición de home
    pd = np.array([0.5, 0.5, 0.5]) # Posicion inicial de la esfera AZUL 
    bmarker_des.xyz([0.5, 0.5, 0.5]) # Donde tiene que llegar
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        if moveObject:
            bmarker_des.xyz(meters2milimiters(get_current_posx()[0]))
        pass
    print('ejecución finalizada')
