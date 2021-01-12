#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import time
import math
# import numpy as np
# import tensorflow as tf

try:
    from pathlib import Path # Python 3 only
except:
    pass


DEBUG = False

try:
    import cv2
    import rospy
    from tm_msgs.srv import SetIO, SetIORequest, SendScript, SendScriptRequest

#    from tm_msgs.msg import * # unnecessary, don't pollute namespaces
#    from tm_msgs.srv import *

except Exception as e:
    print(e)
    # print("Failed to load rospy\n")
    DEBUG = True
    print("DEBUG MODE ENABLED")

if DEBUG:
    print("\nPython version: %s\n" % sys.version)

### QUESTION: Are there other needed packages besides catkin_pkg?
# sys.path.append('/home/robotics/catkin_ws/devel/lib/python2.7/dist-packages')


DEFAULT_X = 350
DEFAULT_Y = 350
DEFAULT_Z = 350
DEFAULT_RX = 180
DEFAULT_RY = 0
DEFAULT_RZ = 135

DEFAULT_WAIT = 5 # used in python sleep function


def take_pic(wait_time=10, fname="temp", save_path="./images"):
    """Takes photo of robot and surroundings, (See toggle_camera, __move)

    TODO: test that this works
    """
    time.sleep(wait_time)
    # Prepare filename and path
    fname += ".png"
    if sys.version_info.major == 3:
        save_path = Path(save_path)
        fname = save_path/fname
    else:
        fname = save_path + "/" + fname
    if not os.path.isdir(save_path):
        os.mkdir(save_path)
    # Take and save picture
    cam = cv2.VideoCapture(0)
    ret, frame = cam.read()
    cv2.imwrite(str(fname), frame)
    cam.release()
    cv2.destroyAllWindows()
    print("Saving picture to: %s" % fname)


class Arm:
    """
    Robot Arm Class

    Commands:

    move - moves arm to given position
        optional parameters: X, Y, Z, Rx, Ry, Rz

    center - centers the arm

    away - moves arm out of camera view

    move_up - moves arm up, (default: 300)
        optional parameter: Z

    move_down - moves arm down, (default: 150)
        optional parameter: Z

    grab - closes gripper
        optional parameter: Z

    release - opens gripper
        optional parameter: Z

    display - prints out stored arm position

    toggle_camera - toggles camera on and off, (takes photos between moves if on)
        optional parameter: wait_time (time to wait between moves)

    """
    def __init__(self, X=350, Y=350, Z=350, Rx=180, Ry=0, Rz=135, debug=False, gripper_open=True, use_killswitch=True):
        """Initialize parameters

        X, Y, Z, Rx, Ry, Rz  - starting position
        debug - If True, prints information instead of moving robot
        gripper_open - Set to False if gripper is initially closed
        """
        self.debug_flag = debug # if set to true, robot won't actually move
        self.__set_coords(X, Y, Z, Rx, Ry, Rz)
        self.__safety_check()
        self.__use_camera = False # if True, take photos in between moves, (See toggle_camera)
        self.__wait_time = 0 # wait time if use_camera set to True, (See toggle_camera)
        self.__previous_script = "" # previous script sent to robot, (See __move)
        self.__gripper_status = gripper_open # True: open, False: closed (See grab and release)
        self.__img_tag = 0
        self.use_killswitch = use_killswitch
        self.__joint_mode = False
        self.__joint1 = 45
        self.__joint2 = 0
        self.__joint3 = 90
        self.__joint4 = 0
        self.__joint5 = 90
        self.__joint6 = 0

    def get_joints(self):
        return [self.__joint1, self.__joint2, self.__joint3, self.__joint4, self.__joint5, self.__joint6]

    def get_coords(self):
        return [self.__X, self__Y, self.__Z, self.__Rx, self.__Ry, self.__Rz]

    def __take_pic(self, fname="temp", save_path="./images", img_tag=None):
        "Takes photo of robot and surroundings, (See toggle_camera, __move)"
        time.sleep(self.__wait_time)
        # Prepare filename and path
        if img_tag is None:
            fname += str(self.__img_tag)
            self.__img_tag += 1
        fname += ".png"
        if sys.version_info.major == 3:
            save_path = Path(save_path)
            fname = save_path/fname
        else:
            fname = save_path + "/" + fname
        if not os.path.isdir(save_path):
            os.mkdir(save_path)
        # Take and save picture
        if not self.debug_flag:
            cam = cv2.VideoCapture(0)
            ret, frame = cam.read()
            cv2.imwrite(str(fname), frame)
            cam.release()
            cv2.destroyAllWindows()
        print("Saving picture to: %s" % fname)

    def __set_coords(self, X, Y, Z, Rx, Ry, Rz):
        "Stores coordinates, (prepares to move arm to given position)"
        for i in [X, Y, Z, Rx, Ry, Rz]:
            if not ((type(i) == int) or (type(i) == float)):
                raise TypeError("Inputs must be integers or floats")
        self.__X = float(X)
        self.__Y = float(Y)
        self.__Z = float(Z)
        self.__Rx = float(Rx)
        self.__Ry = float(Ry)
        self.__Rz = float(Rz)

    def __safety_check(self):
        "Ensures stored coordinates are legal"
        if (self.__Rz <= 45):
            assert self.__X + self.__Y <= 900
        else:
            assert self.__X + self.__Y <= 800
        assert math.sqrt(self.__X**2 + self.__Y**2) >= 282
        assert self.__Z >= 150
        assert self.__X >= -250
        for i in [self.__Rx, self.__Ry, self.__Rz]:
            # assert i >= 0
            assert i < 360

    def __move(self):
        "Moves to stored coordinates"
        self.__joint_mode = False
        self.__safety_check()
        T = [self.__X, self.__Y, self.__Z, self.__Rx, self.__Ry, self.__Rz]
        T = [round(float(t), 2) for t in T]
        T = ["{:.2f}".format(t) for t in T]
        target = ", ".join(T)
        #script = "PTP(\"CPP\", " + target + ", 100, 200, 0, false)"
        script = "PTP(\"CPP\", " + target + ", 100, 0, 0, false)"
        if (not self.debug_flag) and (script != self.__previous_script):
            self.__send_script(script)
            self.__killswitch()
            # self.__run_safe()
        elif script != self.__previous_script:
            print("Script: " + script + "\n")
        if self.__use_camera:
            self.__take_pic()
        self.__previous_script = script

    def __move_angle_dangerous(self):
        "Moves to stored angles"
        # self.__safety_check()
        self.__joint_mode = True
        T = [self.__joint1, self.__joint2, self.__joint3, self.__joint4, self.__joint5, self.__joint6]
        T = [round(float(t), 2) for t in T]
        T = ["{:.2f}".format(t) for t in T]
        target = ", ".join(T)
        script = "PTP(\"JPP\", " + target + ", 100, 0, 0, false)"
        if (not self.debug_flag) and (script != self.__previous_script):
            self.__send_script(script)
            self.__killswitch()
        elif script != self.__previous_script:
            print("Script: " + script + "\n")
        if self.__use_camera:
            self.__take_pic()
        self.__previous_script = script


# 45,0,90,0,90,0
    def move_joints_dangerous(self, joint1=None, joint2=None, joint3=None, joint4=None, joint5=None, joint6=None):
        "Moves arm's individual joints; TODO: SAFETY CHECK FOR THIS"
        if self.debug_flag:
            print("Move by joints")
        if joint1 is not None:
            self.__joint1 = self.__assert_num(joint1)
        if joint2 is not None:
            self.__joint2 = self.__assert_num(joint2)
        if joint3 is not None:
            self.__joint3 = self.__assert_num(joint3)
        if joint4 is not None:
            self.__joint4 = self.__assert_num(joint4)
        if joint5 is not None:
            self.__joint5 = self.__assert_num(joint5)
        if joint6 is not None:
            self.__joint6 = self.__assert_num(joint6)
        self.__move_angle_dangerous()

    def __killswitch(self):
        "Allows user to stop robot"
        if self.use_killswitch:
            # k = input("Hit k+[ENTER] to kill robot command;\nHit p+[ENTER] to pause\n hit (any key) + [ENTER] to continue ")
            k = input("[ENTER] - kill execution\n(any key) + [ENTER] - continue ")
            if k == "":
                self.__send_script("QueueTag()")
                # self.pause()
                print("\nKilled\n")
                exit(0)

    def wait(self):
        resp = self.__send_script("QueueTag(13,1)")
        print(resp)
        return



    def pause(self):
        "Pauses robot movement"
        self.__send_script("Pause()")

    def resume(self):
        "Resumes robot movement"
        self.__send_script("Resume()")

    @staticmethod
    def __send_script(script):
        "Controls robot, (See __move)"
        rospy.wait_for_service('/tm_driver/send_script')
        try:
            script_service = rospy.ServiceProxy('/tm_driver/send_script', SendScript)
            move_cmd = SendScriptRequest()
            move_cmd.script = script
            resp = script_service(move_cmd)
            return resp
        except rospy.ServiceException as e:
            print("Send script service call failed: %s"%e)

    @staticmethod
    def __set_io(state):
        "Controls robot, (See grab and release)"
        rospy.wait_for_service('/tm_driver/set_io')
        try:
            io_service = rospy.ServiceProxy('/tm_driver/set_io', SetIO)
            io_cmd = SetIORequest()
            io_cmd.module = 1
            io_cmd.type = 1
            io_cmd.pin = 0
            io_cmd.state = state
            io_service(io_cmd)
        except rospy.ServiceException as e:
            print("IO service call failed: %s" % e)

    @staticmethod
    def __assert_num(n):
        assert ((type(n) == float) or (type(n) == int))
        return float(n)

    def move_down(self, Z=150.0):
        "Move arm down, (optional parameter: Z)"
        if self.debug_flag:
            print("Move down")
        assert (type(Z) == float) or (type(Z) == int)
        self.__Z = float(Z)
        self.__move()

    def move_up(self, Z=300):
        "Move arm up, (optional parameter: Z)"
        if self.debug_flag:
            print("Move up")
        assert (type(Z) == float) or (type(Z) == int)
        self.__Z = float(Z)
        self.__move()

    def move(self, X=None, Y=None, Z=None, Rx=None, Ry=None, Rz=None):
        "Move to a given position"
        if self.debug_flag:
            print("Move")
        if X is not None:
            self.__X = self.__assert_num(X)
        if Y is not None:
            self.__Y = self.__assert_num(Y)
        if Z is not None:
            self.__Z = self.__assert_num(Z)
        if Rx is not None:
            self.__Rx = self.__assert_num(Rx)
        if Ry is not None:
            self.__Ry = self.__assert_num(Ry)
        if Rz is not None:
            self.__Rz = self.__assert_num(Rz)
        self.__move()




    def center(self):
        "Centers the arm, (move to position: 350, 350, 250, 180, 0, 135)"
        self.move(350, 350, 450, 180, 0, 135)

    def away(self):
        self.move(-200, 400, 450, 180, 0, 135)


    def grab(self, Z=None):
        "Closes gripper at Z (if given) or at current position"
        if self.__gripper_status == False:
            # Check if gripper already closed
            if self.debug_flag:
                print("Gripper is already closed")
            return
        else:
            self.__gripper_status = False
        if self.debug_flag:
            print("Grab")
        if Z is not None:
            self.move_down(Z)
        if not self.debug_flag:
            self.__set_io(1.0)
        time.sleep(1)

    def release(self, Z=None):
        "Opens gripper from Z (if given) or at current position"
        if self.__gripper_status == True:
            # Check if gripper already open
            if self.debug_flag:
                print("Gripper is already open")
            return
        else:
            self.__gripper_status = True
        if self.debug_flag:
            print("Release")
        if Z is not None:
            self.move_up(Z)
        if not self.debug_flag:
            self.__set_io(0.0)
        time.sleep(2)

    def toggle_camera(self, wait_time=None):
        """
        Toggles camera on or off (which takes photos between moves)
        Sets wait_time if given
        """
        if wait_time is not None:
            self.__wait_time = wait_time
        self.__use_camera = not self.__use_camera
        if self.debug_flag:
            print("Toggle Camera " + ("On" if self.__use_camera else "Off"))

    def display(self):
        "Prints current stored coordinates"
        if self.__joint_mode == False:
            print("\nX: {:.2f}".format(self.__X))
            print("Y: {:.2f}".format(self.__Y))
            print("Z: {:.2f}".format(self.__Z))
            print("Rx: {:.2f}".format(self.__Rx))
            print("Ry: {:.2f}".format(self.__Ry))
            print("Rz: {:.2f}".format(self.__Rz))
            print("Camera: " + ("On" if self.__use_camera else "Off"))
            if self.__use_camera:
                print("Wait time: %s"% self.__wait_time)
            print("Debug mode: " + ("Yes" if self.debug_flag else "No"))
            print("")
        else:
            print("\njoint1: {:.2f}".format(self.__joint1))
            print("joint2: {:.2f}".format(self.__joint2))
            print("joint3: {:.2f}".format(self.__joint3))
            print("joint4: {:.2f}".format(self.__joint4))
            print("joint5: {:.2f}".format(self.__joint5))
            print("joint6: {:.2f}".format(self.__joint6))
            print("Camera: " + ("On" if self.__use_camera else "Off"))
            if self.__use_camera:
                print("Wait time: %s"% self.__wait_time)
            print("Debug mode: " + ("Yes" if self.debug_flag else "No"))
            print("")



    def rotate(self, Rx=None, Ry=None, Rz=None):
        "Rotates given axes individually"
        if Rx is not None:
            self.__Rx = Rx
            self.__move()
        if Ry is not None:
            self.__Ry = Ry
            self.__move()
        if Rz is not None:
            self.__Rz = Rz
            self.__move()

    def rotate_X(self, amt):
        "Rotates Rx by given amount"
        self.__Rx += amt
        if self.__Rx > 360:
            self.__Rx -= 360
        self.__move()

    def rotate_Y(self, amt):
        "Rotates Ry by given amount"
        self.__Ry += amt
        if self.__Ry > 360:
            self.__Ry -= 360
        self.__move()

    def rotate_Z(self, amt):
        "Rotates Rz by given amount"
        self.__Rz += amt
        if self.__Rz > 360:
            self.__Rz -= 360
        self.__move()





if __name__ == "__main__":



    try:
        if DEBUG:
            rospy = None
        if not DEBUG:
            rospy.init_node('send_scripts', anonymous=True)

        if len(sys.argv) >= 7:
            x,y,z,rx,ry,rz = [float(arg) for arg in sys.argv[1:7]]


        # Example Usage

        # arm = Arm(X=300,Y=300,Z=350,Rx=180,Ry=0,Rz=0)
        arm = Arm(use_killswitch=False)
        #arm = Arm(DEFAULT_X, DEFAULT_Y, DEFAULT_Z, DEFAULT_RX, DEFAULT_RY, DEFAULT_RZ)
        #arm.debug_flag = DEBUG # If True, robot won't actually move
        #arm.center()
        #arm.move(X=150, Y=550)
        #arm.center()
        #print('arm center')
        arm.away()
        arm.wait()
        arm.center()

        #arm.display()
        #arm.move(X=300, Y=300)
        # arm.move(Rx=160, Ry=0, Rz=0)

        #arm.rotate_Y(10)

        # arm.away()
        # arm.display()
#
        # arm.move(Rx=100)
        # time.sleep(5)
        # arm.display()
#
        # arm.move(Ry=70)
        # time.sleep(5)
        # arm.display()
#
        # arm.move(Rz=180)
        # time.sleep(5)
        # arm.display()


        # XYs = [
            # [400, 50],
            # [225, 225],
            # [50, 400],
            # [550, 150],
            # [350, 350],
            # [150, 550],
            # [600, 200],
            # [400, 400],
            # [200, 600],
            # [500, 100],
            # [300, 300],
            # [100, 500],
        # ]


        # for x,y in XYs:
            # print(f"X: {x}, Y: {y}\n")
            # arm.move(X=x, Y=y)
            # arm.move_down(155)
            # time.sleep(25)
            # arm.move_up()
            # arm.away()
            # take_pic(fname=f"img_{x}_{y}")



        #arm.move()
        #arm.toggle_camera()
        # arm.move(X=350, Y=200, Z=355)

        # arm.move() # move arm to default position
        # arm.move() # nothing happens if position has not changed
        # arm.move()
        # arm.display() # display arm parameters
        # arm.move_down() # move arm down to Z = 150
        # arm.grab() # close the gripper
        # arm.move_up() # move arm up to default Z
        # arm.release() # open the gripper
        # arm.toggle_camera(wait_time=DEFAULT_WAIT) # toggle camera on
#
        if len(sys.argv) >= 7:
            arm.move(x,y,z,rx,ry,rz)
            # arm.move(Rx=rx,Ry=ry,Rz=rz) # move() can accept a variable number of parameters
            # arm.display()
            # arm.toggle_camera() # toggle camera off
#
        # arm.display()
        # arm.center() # move back to center

#    except rospy.ROSInterruptException:
    except Exception as e:
        if e is not rospy.ROSInterruptException:
            print(e)
