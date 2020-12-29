#!/usr/bin/env python3

import math
import cv2
import numpy as np
from get_pixel_position import get_pixel_position
from arm_class import Arm
import time


# For cutting
# joint3 -10
# joint4 +20

def main():
    arm = Arm()

    # get knife
    arm.move(X=460, Y=140)
    arm.grab(168)
    arm.move_up()

    step_size = 10
    x = 250
    y = 450
    for _ in range(5):
        # prepare to cut
        arm.move(X=x, Y=y, Rx=200)
        arm.move_down(190)

        # cut
        arm.move(X=x-100, Y=y-100)
        arm.move_up(250)

        # move back
        arm.move(X=x, Y=y)
        x -= step_size
        y += step_size


    # return knife
    arm.move_up()
    arm.move(Rx=180)
    arm.move(X=460, Y=140)
    arm.move_down(168)
    arm.release()
    arm.move_up()
    arm.center()



    # prepare to cut
    # arm.move(X=250, Y=450, Rx=200)
    # arm.move_down(200)

    # cut
    # arm.move(X=150, Y=350)


    # gripperHomeX = 340
    # gripperHomeY = 230
    # arm = Arm(X=150, Y=550, Z=350, Rx=180, Ry=0, Rz=135)
    # arm.release()
    # arm.use_killswitch = True
    # arm.move(Z=400)
    # arm.move(Rx=135, Ry=0, Rz=135)
    # arm.move(X=gripperHomeX,Y=gripperHomeY)
    # arm.move(Z=310)
    # arm.grab()
    # arm.move(Z=400)
    # arm.release()
    # arm.move(Y=130)
    # arm.move(Z=320)
    # arm.grab()
    # arm.move(Z=400)

    # # move block
    # arm.move(X=310,Y=100)
    # arm.move(Z=320)
    # arm.release()
    # arm.move(Z=400)

    # # return tool
    # arm.move(X=gripperHomeX,Y=gripperHomeY)
    # arm.grab()
    # arm.move(Z=310)
    # arm.release()
    # arm.move(Z=400)
    # arm.display()


    # arm.move()
    # arm.move(X=530, Y=170, Z=400)
    # arm.rotate_Y(90)
    # arm.rotate_X(-10)
    # arm.rotate_X(10)
    # arm.rotate_Z(10)
    # arm.rotate_Z(-10)
    # arm.rotate_Y(-90)

    # arm.grab()
    # arm.move(Z=450)
    # arm.move(X=350, Y=350)
    # arm.release()
    # arm.move(Z=400)

    # arm.move_down(Z = 170)
    # arm.center()
    # arm.grab()
    # arm.move_up(Z = 400)
    # arm.move_down(Z = 170)
    # arm.release()
    # arm.away()
    # arm.release()
    # arm.move_joints_dangerous(60, -5, 108, -12, 92, 15) # 45, 0, 90, 0, 90, 0
    # arm.debug_flag = True
    # arm.move_joints_dangerous(90,0,80,15,90,0) # 50
    # arm.move_joints_dangerous(90,0,90,0,90,0)
    # arm.move(X=150, Y=550)
    # arm.move_joints_dangerous(joint1=90) # leftside


if __name__ == "__main__":
    main()
