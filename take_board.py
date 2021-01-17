import numpy as np
import re
import copy
import math
import cv2
from get_pixel_position import get_pixel_position
from arm_class import Arm
import time
from motion_generator import Motion, MotionCommand, GetSpecialMotion, PrintRawMCList
from hsv import get_blue_orange, get_cuts, get_food_positions, get_cutting_board_hull
from test1 import cut_proc


def tilt_board(arm):
    arm.move_joints_dangerous(joint1=90,joint2=36,joint3=70,joint4=70,joint5=90,joint6=90)

    # arm.grab()

    arm.move_joints_dangerous(joint1=110)

    # j6 128 --> 140

    arm.move_joints_dangerous(joint6=140)

    # arm.move_joints_dangerous(joint1=116)
    # arm.move_joints_dangerous(joint1=120)
    # arm.move_joints_dangerous(joint6=120)
    # arm.move_joints_dangerous(joint1=90)

def hull_center(hull):
    longest_dist = 0
    current_longest = (-1,-1)
    for i in range(len(hull)):
        for j in range(i+1, len(hull)):
            dist = np.linalg.norm(hull[i]-hull[j])
            if dist > longest_dist:
                longest_dist = dist
                current_longest = (i,j)
    return (hull[i]+hull[j])//2

def board_proc(arm):
    "grabs board, dumps contents, returns board"

    arm.away()
    arm.wait()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("not opened")
    ret,img = cap.read()
    hull = get_cutting_board_hull(img)
    board_center = hull_center(hull)
    candidate_edge_points = []
    for i in range(len(hull)):
        j = (i+1)%len(hull)
        if board_center[0] < hull[i][0] and np.abs(hull[i][0]-hull[j][0]) < np.abs(hull[i][1]-hull[j][1])*0.2:
            candidate_edge_points.append((hull[i]+hull[j])//2)
            candidate_edge_points.append(hull[i])
            candidate_edge_points.append(hull[j])
    
    candidate_edge_points.sort(key=lambda x:x[1])
    board_grabbing_point = (candidate_edge_points[0] + candidate_edge_points[len(candidate_edge_points)-1])//2
    

    draw_res = img.copy()
    for c in hull:
        cv2.circle(draw_res, (int(c[0]),int(c[1])), 3, (255,255,255), -1)
    for c in candidate_edge_points:
        cv2.circle(draw_res, (int(c[0]),int(c[1])), 3, (0,255,0), -1)
    cv2.circle(draw_res, (board_grabbing_point[0],board_grabbing_point[1]), 3, (255,255,0), -1)
    cv2.imshow('board grabbing', draw_res)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    board_z = 110
    arm.release()
    arm.center()

    # grab board
    arm.move(Ry=90)
    arm.move(X=110,Y =590)
    arm.move_down(150)
    arm.move_down(board_z)
    arm.move(X=130,Y =570)
    arm.grab()
    arm.move_up()
    arm.move(X=130,Y =570)

    # tilt board
    arm.move_joints_dangerous(joint1=90,joint2=36,joint3=70,joint4=70,joint5=90,joint6=90)
    arm.move_joints_dangerous(joint1=110)
    arm.move_joints_dangerous(joint6=140)

    # return board
    arm.move(X=130,Y =570)
    arm.move_down(board_z)
    arm.release()
    arm.move(X=110,Y =590)
    arm.move_up()
    


def main():
    # board_z = 110

    # arm = Arm(X=340,Y=340,Z=450,Rx=180,Ry=0,Rz=135,gripper_open=False, use_killswitch=False)
    arm = Arm(X=340,Y=340,Z=450,Rx=180,Ry=0,Rz=135,gripper_open=False, use_killswitch=True)

    # cut_proc(arm)

    board_proc(arm)


    # arm.away()
    # arm.release()
    # arm.center()
    # arm.move(Ry=90)
    # arm.move(X=110,Y =590)
    # arm.move_down(150)
    # arm.move_down(board_z)
    # arm.move(X=130,Y =570)
    # arm.grab()
    # arm.move_up()
    # arm.move(X=130,Y =570)

    # tilt_board(arm)

    # arm.move(X=130,Y =570)
    # arm.move_down(board_z)
    # arm.release()
    # arm.move(X=110,Y =590)
    # arm.move_up()

    # # arm.move(Rz=85)
    # # for rz in range(135,45,-5):
    # #     print(rz)
    # #     arm.move(Rz=rz)



    # while True:
    #     line = input('input stuff: ')
    #     args = [float(x) for x in line.split()]
    #     print(args)
    #     asdf = input("proceed?")
    #     if ("n" in asdf) or ("N" in asdf):
    #         break
    #     arm.move(*args)
        
    # arm.center()
    # return


if __name__ == "__main__":
    main()