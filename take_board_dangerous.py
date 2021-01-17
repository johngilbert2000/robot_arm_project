import numpy as np
import re
import copy
import math
import cv2
from get_pixel_position import get_pixel_position
from arm_class import Arm
import time
from motion_generator import Motion, MotionCommand, GetSpecialMotion, PrintRawMCList
from hsv import get_blue_orange, get_cuts, get_food_positions
from test1 import cut_proc

def main():
    # arm = Arm(X=340,Y=340,Z=450,Rx=180,Ry=0,Rz=135,gripper_open=False, use_killswitch=False)
    arm = Arm(X=340,Y=340,Z=450,Rx=180,Ry=0,Rz=135,gripper_open=False, use_killswitch=True)
    # arm.away()
    arm.release()
    # arm.center()

# j1=45
#     arm.move_joints_dangerous(45,1,90,1,90,1)

#     arm.move_joints_dangerous(joint1=90)

#     arm.move_joints_dangerous(joint6=40)


# # j4: 65 ~ 80

#     arm.move_joints_dangerous(joint4=65)

#     arm.move_joints_dangerous(joint6=90)

#     # for x in range(60,100,2):
#     #     print("j6,",x)
#     #     arm.move_joints_dangerous(joint6=x)


#     arm.move_joints_dangerous(joint2=16)

#     arm.display()

# # j3 get smaller.... 70
#     arm.move_joints_dangerous(joint3=70)
    # arm.center()

    arm.move_joints_dangerous(joint1=90,joint2=36,joint3=70,joint4=70,joint5=90,joint6=90)

    # arm.grab()

    arm.move_joints_dangerous(joint1=110)

    # j6 128 --> 140

    arm.move_joints_dangerous(joint6=140)

    arm.move_joints_dangerous(joint1=116)
    arm.move_joints_dangerous(joint1=120)
    arm.move_joints_dangerous(joint6=120)
    arm.move_joints_dangerous(joint1=90)


    arm.display()

    # for i in range(110,180,2):
    #     print("j6",i)
    #     arm.move_joints_dangerous(joint6=i)

    # for x in range()


    # for x in range(90,44,-2):
    #     print("j3",x)
    #     arm.move_joints_dangerous(joint3=x)


    # for x in range(16,90,2):
    #     print("j2,",x)
    #     arm.move_joints_dangerous(joint2=x)

    # for x in range(60,100,2):
    #     print("j4",x)
    #     arm.move_joints_dangerous(joint4=x)

    # arm.center()


    # print("joint5")
    # arm.move_joints_dangerous(joint5=95)
    # arm.move_joints_dangerous(joint5=90)
    # print("joint4")
    # arm.move_joints_dangerous(joint5=-5)
    # arm.move_joints_dangerous(joint5=-1)

    # print("joint3")
    # arm.move_joints_dangerous(joint3=95)
    # arm.move_joints_dangerous(joint3=90)

    # print("joint2")
    # arm.move_joints_dangerous(joint2=5)
    # arm.move_joints_dangerous(joint2=1)









    # arm.move(Ry=90)
    # arm.move(X=110,Y =590)
    # arm.move_down(150)
    # arm.move_down(110)
    # arm.move(X=130,Y =570)
    # arm.grab()
    # arm.move_up()
    # arm.move(X=130,Y =570)

    # arm.move(Rz=85)

    # arm.rotate_X(10)
    # arm.rotate_X(-20)
    # arm.rotate_X(10)

    # arm.rotate_Y(10)
    # arm.rotate_Y(-20)
    # arm.rotate_Y(10)



    # while True:
    #     line = input('input stuff: ')
    #     args = [float(x) for x in line.split()]
    #     print(args)
    #     asdf = input("proceed?")
    #     if ("n" in asdf) or ("N" in asdf):
    #         break
    #     arm.move(*args)

    # arm.center()
    return


if __name__ == "__main__":
    main()
