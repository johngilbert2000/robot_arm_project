#!/usr/bin/env python

import sys
sys.path.append('/home/robotics/catkin_ws/devel/lib/python2.7/dist-packages')

import rospy
from tm_msgs.msg import *
from tm_msgs.srv import *
def send_script(script):
    rospy.wait_for_service('/tm_driver/send_script')
    try:
        script_service = rospy.ServiceProxy('/tm_driver/send_script', SendScript)
        move_cmd = SendScriptRequest()
        move_cmd.script = script
        resp1 = script_service(move_cmd)
    except rospy.ServiceException as e:
        print("Send script service call failed: %s"%e)

def set_io(state):
    rospy.wait_for_service('/tm_driver/set_io')
    try:
        io_service = rospy.ServiceProxy('/tm_driver/set_io', SetIO)
        io_cmd = SetIORequest()
        io_cmd.module = 1
        io_cmd.type = 1
        io_cmd.pin = 0
        io_cmd.state = state
        resp1 = io_service(io_cmd)
    except rospy.ServiceException as e:
        print("IO service call failed: %s"%e)

if __name__ == '__main__':
    try:
        #--- move command by joint angle ---#
        # script = 'PTP(\"JPP\",45,0,90,0,90,0,35,200,0,false)'
        rospy.init_node('send_scripts', anonymous=True)
        #--- move command by end effector's pose (x,y,z,rx,ry,rz) ---#
        targetP1 = "200.00 , 200.00 , 500.00 , 180.00 , 0.00 , 135.00"
        script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"

        send_script(script)
        set_io(0.0)# 1.0: close gripper, 0.0: open gripper
        
        set_io(1.0)# 1.0: close gripper, 0.0: open gripper
        #set_io(0.0)# 1.0: close gripper, 0.0: open gripper
        
    except rospy.ROSInterruptException:
        pass
