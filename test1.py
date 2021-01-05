import numpy as np
import re
import copy
import math
import cv2
from get_pixel_position import get_pixel_position
from arm_class import Arm
import time
from motion_generator import Motion, MotionCommand, GetSpecialMotion, PrintRawMCList
from hsv import get_blue_orange, get_cuts


def get_tooltip_offset(Rx, Rz, tool_length = 100):
    RxRad = np.deg2rad(Rx)
    r = np.sin(RxRad)*tool_length
    h = np.cos(RxRad)*tool_length
    RzRadMinus90 = np.deg2rad(Rz-90)
    x = np.cos(RzRadMinus90)*r
    y = np.sin(RzRadMinus90)*r
    return float(x),float(y),float(h)

def get_gripper_xy(blue_xy, orange_xy):
    unit_x = orange_xy - blue_xy
    unit_x /= np.linalg.norm(unit_x)
    unit_y = np.array([-unit_x[1], unit_x[0]])
    ret = blue_xy + 172*unit_x + 48*unit_y
    return ret

def get_knife_xy(blue_xy, orange_xy):
    unit_x = orange_xy - blue_xy
    unit_x /= np.linalg.norm(unit_x)
    unit_y = np.array([-unit_x[1], unit_x[0]])
    ret = blue_xy -22*unit_x + 38*unit_y
    return ret

def get_rz(blue_xy, orange_xy):
    unit_x = orange_xy - blue_xy
    rz = np.rad2deg(np.arctan2(unit_x[1], unit_x[0]))+90
    return float(rz)


def plan_motion():
    #start = [MotionCommand("move_immutable",{'X':340,'Y':340,'Z':450,'Rx':180,'Ry':0,'Rz':135})]

    #m = Motion(start)
    #return m



    timestamp = int(time.time())
    output_img_fn = 'images/image-%d.png' % timestamp

    cap = cv2.VideoCapture(0)
    ret,frame = cap.read()
    cv2.imwrite(output_img_fn, frame)

    blue_c, orange_c = get_blue_orange(frame, debug=False)

    fucking_camera_offset = np.array([8,22])

    stand_z = 172
    blue_x, blue_y = get_pixel_position(blue_c[0], blue_c[1], stand_z)
    orange_x, orange_y = get_pixel_position(orange_c[0], orange_c[1], stand_z)
    blue_xy = np.array([blue_x, blue_y])
    orange_xy = np.array([orange_x, orange_y])

    blue_xy += fucking_camera_offset
    orange_xy += fucking_camera_offset


    gripper_xy = get_gripper_xy(blue_xy, orange_xy)
    knife_xy = get_knife_xy(blue_xy, orange_xy)
    rz = get_rz(blue_xy, orange_xy)

    dist = np.linalg.norm(blue_xy-orange_xy)
    print(blue_x, blue_y)
    print(orange_x, orange_y)
    print(dist)
    print('gripper:', gripper_xy)
    print('knife:', knife_xy)
    print('rz:', rz)


    print('blue:')
    print(blue_c)
    print('orange:')
    print(orange_c)

    cap.release()


    start = [MotionCommand("move_immutable",{'X':340,'Y':340,'Z':450,'Rx':180,'Ry':0,'Rz':135})]
    end = [MotionCommand("move_immutable",{'X':340,'Y':340,'Z':450,'Rx':180,'Ry':0,'Rz':135})]
    away = [MotionCommand("away",{})]


    gripper_rx = 135
    gripper_tooltip_offset = get_tooltip_offset(gripper_rx, rz)
    gripper_position_z = float(244 - gripper_tooltip_offset[2])

    take_gripper_safe = {'Z':450}
    take_gripper_rotation = {'Rx':gripper_rx,'Ry':0}
    take_gripper_position_pre = {'Z':gripper_position_z + 40}
    take_gripper_position = {'Z':gripper_position_z}
    take_gripper_motion = [MotionCommand("move", take_gripper_safe),
        MotionCommand("release", {}),
        MotionCommand("move", take_gripper_rotation),
        MotionCommand("move", take_gripper_position_pre),
        MotionCommand("move", take_gripper_position),
        MotionCommand("grab", {}),
        MotionCommand("move", take_gripper_safe),
        MotionCommand("release", {})]
    put_gripper_safe = {'Z':450}
    put_gripper_rotation = {'Rx':gripper_rx,'Ry':0}
    put_gripper_position = {'Z':gripper_position_z}
    put_gripper_motion = [MotionCommand("move", put_gripper_safe),
        MotionCommand("grab", {}),
        MotionCommand("move", put_gripper_rotation),
        MotionCommand("move", put_gripper_position),
        MotionCommand("release", {}),
        MotionCommand("move", put_gripper_safe),
        MotionCommand("move", {"Rx": 180})]
    take_knife_safe = {'Z':450}
    take_knife_rotation = {'Rx':180,'Ry':0}
    take_knife_position_pre_pre = {'Z': 215}
    take_knife_position_pre = {'Z': 200}
    take_knife_position = {'Z':192}
    take_knife_motion = [MotionCommand("move", take_knife_safe),
        MotionCommand("release", {}),
        MotionCommand("move", take_knife_rotation),
        MotionCommand("move", take_knife_position_pre_pre),
        MotionCommand("move", take_knife_position_pre),
        MotionCommand("move", take_knife_position),
        MotionCommand("grab", {}),
        MotionCommand("move", take_knife_safe)]
    put_knife_safe = {'Z':450}
    put_knife_rotation = {'Rx':180,'Ry':0}
    put_knife_position_pre = {'Z': 200}
    put_knife_position = {'Z':192}
    put_knife_motion = [MotionCommand("move", put_knife_safe),
        MotionCommand("move", put_knife_rotation),
        MotionCommand("move", put_knife_position_pre),
        MotionCommand("move", put_knife_position),
        MotionCommand("release", {}),
        MotionCommand("move", put_knife_safe)]

    gripper_xy = gripper_xy - np.array([gripper_tooltip_offset[i] for i in range(2)])
    gripper_location = MotionCommand("move",{'X':float(gripper_xy[0]),'Y':float(gripper_xy[1]),'Rz':rz})

    take_gripper = [gripper_location]+take_gripper_motion
    put_gripper = [gripper_location]+put_gripper_motion

    knife_location = MotionCommand("move",{'X':float(knife_xy[0]),'Y':float(knife_xy[1]),'Rz':rz})
    take_knife = [knife_location]+take_knife_motion
    put_knife = [knife_location]+put_knife_motion


    item_location = MotionCommand("move",{'X':150,'Y':400,'Rz':135})
    item_grab = [MotionCommand("move",{'Z':320}),MotionCommand("grab",{}),MotionCommand("move",{'Z':450})]
    new_item_location = MotionCommand("move",{'X':180,'Y':370,'Rz':100})
    item_release = [MotionCommand("move",{'Z':320}),MotionCommand("release",{}),MotionCommand("move",{'Z':450})]
    item_subroutine = [item_location]+item_grab+[new_item_location]+item_release+[MotionCommand("move",{'Z':450})]

    cut_location = [MotionCommand("move",{'X':200,'Y':450,'Z':450})]
    knife_subroutine = []
    knife_rotate = [MotionCommand("move", {'Ry':10})]
    knife_rotate_back = [MotionCommand("move", {'Ry':0})]
    for i in [245,242,239,236,233]:
        knife_down = [MotionCommand("move", {'Z':275}), MotionCommand("move", {'Z':i})]
        knife_subroutine.extend(knife_down + knife_rotate + knife_rotate_back + cut_location)
    cut = cut_location + knife_subroutine


    #MCList = start + take_gripper + item_subroutine + put_gripper + take_knife + put_knife + end
    #MCList = start + away + end
    #MCList = start + take_knife + away + put_knife + end
    MCList = start + take_knife + cut + put_knife + end

    print('Raw Motion Commands:')
    PrintRawMCList(MCList)
    print('\n\n=======================\n\n')
    m = Motion(MCList)
    print('Decoded Motion:')
    print(m)
    return m

def execute_motion_dangerous(m, arm):
    for i in range(len(m.MCList)):
        MC = m.MCList[i]
        print(MC)
        if MC.typename in ['move','move_immutable']:
            arm.move(X=MC.param['X'], Y=MC.param['Y'], Z=MC.param['Z'], Rx=MC.param['Rx'], Ry=MC.param['Ry'], Rz=MC.param['Rz'])
        elif MC.typename == 'away':
            arm.away()
        elif MC.typename == 'center':
            arm.center()
        elif MC.typename == 'grab':
            arm.grab()
        elif MC.typename == 'release':
            arm.release()
    return


def main():
    #arm = Arm(X=340,Y=340,Z=450,Rx=180,Ry=0,Rz=135,gripper_open=False, _use_killswitch=False)
    arm = Arm(X=340,Y=340,Z=450,Rx=180,Ry=0,Rz=135,gripper_open=False, _use_killswitch=True)
    m = Motion([MotionCommand("away", {}), MotionCommand("release", {})])
    execute_motion_dangerous(m, arm)
    time.sleep(10)
    #m = plan_motion()
    #execute_motion_dangerous(m, arm)
    return


if __name__ == "__main__":
    main()
