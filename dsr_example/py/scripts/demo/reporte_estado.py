#!/usr/bin/env python

import rospy
import os
import threading, time
import sys
from sensor_msgs.msg import Joy
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *


def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if (0==(msgRobotState_cb.count % 100)): 
        rospy.loginfo("________ ROBOT STATUS ________")
        print("  Estado del robot   : %s" % (msg.robot_state_str))
        print("  Actuales angulos de articulacion      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        print("  Error de articulaciones  : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.joint_err[0],msg.joint_err[1],msg.joint_err[2],msg.joint_err[3],msg.joint_err[4],msg.joint_err[5]))
        print("  Actual posicion      : (%7.3f %7.3f %7.3f)" % (msg.current_posx[0],msg.current_posx[1],msg.current_posx[2]))
        print("  Error de posicion  : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.task_err[0],msg.task_err[1],msg.task_err[2],msg.task_err[3],msg.task_err[4],msg.task_err[5]))
        print("  torque dinamico  : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.dynamic_tor[0],msg.dynamic_tor[1],msg.dynamic_tor[2],msg.dynamic_tor[3],msg.dynamic_tor[4],msg.dynamic_tor[5]))
        print("  sensor torque  : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_jts[0],msg.actual_jts[1],msg.actual_jts[2],msg.actual_jts[3],msg.actual_jts[4],msg.actual_jts[5]))
        print("  Fuerza externa sobre las articulaciones  : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.actual_ejt[0],msg.actual_ejt[1],msg.actual_ejt[2],msg.actual_ejt[3],msg.actual_ejt[4],msg.actual_ejt[5]))

msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('reporte_estado_robot')
    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True 
    t1.start()
    #pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)
    #sub_joy  = rospy.Subscriber("joy", Joy, joy_cb)
    while not rospy.is_shutdown():
        pass