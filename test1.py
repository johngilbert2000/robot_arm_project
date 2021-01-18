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

def get_tooltip_offset(Rx, Rz, tool_length = 105):
    RxRad = np.deg2rad(Rx)
    r = np.sin(RxRad)*tool_length
    h = np.cos(RxRad)*tool_length
    RzRadMinus90 = np.deg2rad(Rz-90)
    x = np.cos(RzRadMinus90)*r
    y = np.sin(RzRadMinus90)*r
    return float(x),float(y),float(h)

def get_knifetip_offset(Rz):
    RzRad = np.deg2rad(Rz + 55)
    RzDelta = RzRad

    if Rz > 200 and Rz < 250:
        r = 30
    else:
        r = 55
    x = np.cos(RzDelta)*r
    y = np.sin(RzDelta)*r
    return float(x),float(y)


def get_prong_xy(blue_xy, orange_xy):
    unit_x = orange_xy - blue_xy
    unit_x /= np.linalg.norm(unit_x)
    unit_y = np.array([-unit_x[1], unit_x[0]])
    ret = blue_xy + 165*unit_x + 36*unit_y
    return ret

def get_knife_xy(blue_xy, orange_xy):
    unit_x = orange_xy - blue_xy
    unit_x /= np.linalg.norm(unit_x)
    unit_y = np.array([-unit_x[1], unit_x[0]])
    ret = orange_xy + 33*unit_x + 0*unit_y
    return ret

def get_knife_xy_pre(blue_xy, orange_xy):
    unit_x = orange_xy - blue_xy
    unit_x /= np.linalg.norm(unit_x)
    unit_y = np.array([-unit_x[1], unit_x[0]])
    ret = orange_xy + 10*unit_x + 0*unit_y
    return ret

def get_rz(blue_xy, orange_xy):
    unit_x = orange_xy - blue_xy
    rz = np.rad2deg(np.arctan2(unit_x[1], unit_x[0]))+90
    return float(rz)


def plan_prong_motion():
    timestamp = int(time.time())
    output_img_fn = 'images/image-%d.png' % timestamp

    cap = cv2.VideoCapture(0)
    ret,frame = cap.read()
    cv2.imwrite(output_img_fn, frame)

    valid_centers = get_food_positions(frame, debug=True)
    print(valid_centers)


    blue_c, orange_c = get_blue_orange(frame, debug=False)
    fucking_camera_offset = np.array([8,22])
    stand_z = 172
    blue_x, blue_y = get_pixel_position(blue_c[0], blue_c[1], stand_z)
    orange_x, orange_y = get_pixel_position(orange_c[0], orange_c[1], stand_z)
    blue_xy = np.array([blue_x, blue_y])
    orange_xy = np.array([orange_x, orange_y])
    blue_xy += fucking_camera_offset
    orange_xy += fucking_camera_offset
    prong_xy = get_prong_xy(blue_xy, orange_xy)

    rz = get_rz(blue_xy, orange_xy)
    print('rz:', rz)

    prong_rx = 135
    prong_tooltip_offset = get_tooltip_offset(prong_rx, rz)
    prong_xy = prong_xy - np.array([prong_tooltip_offset[i] for i in range(2)])


    prong_rest = MotionCommand("move",{'X':float(prong_xy[0]),'Y':float(prong_xy[1]),'Rz':rz})
    prong_position_z = float(230 - prong_tooltip_offset[2])

    take_prong_safe = {'Z':450}
    take_prong_rotation = {'Rx':prong_rx,'Ry':0}
    take_prong_position_pre = {'Z':prong_position_z + 55}
    take_prong_position = {'Z':prong_position_z}
    take_prong_motion = [
        MotionCommand("move", take_prong_safe),
        MotionCommand("release", {}),
        prong_rest,
        MotionCommand("move", take_prong_rotation),
        MotionCommand("move", take_prong_position_pre),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("move", take_prong_position),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("wait",{}),
        MotionCommand("grab", {}),
        MotionCommand("wait", {}),
        MotionCommand("move", take_prong_safe),
        MotionCommand("release", {})]
    put_prong_safe = {'Z':450}
    put_prong_rotation = {'Rx':prong_rx,'Ry':0}
    put_prong_position = {'Z':prong_position_z}
    put_prong_motion = [MotionCommand("move", put_prong_safe),
        MotionCommand("grab", {}),
        prong_rest,
        MotionCommand("move", put_prong_rotation),
        MotionCommand("move", put_prong_position),
        MotionCommand("release", {}),
        MotionCommand("wait",{}),
        MotionCommand("move", put_prong_safe),
        MotionCommand("move", {"Rx": 180})]



    prong_motion = []
    prong_safe_height = 450
    cutting_board_z = 235
    prong_motion.append(MotionCommand("move", {'Z':  prong_safe_height}))
    for c in valid_centers:
        item_x, item_y = get_pixel_position(c[0], c[1], cutting_board_z)
        item_x = float(item_x - prong_tooltip_offset[0])
        item_y = float(item_y - prong_tooltip_offset[1])
        item_z = float(407)
        item_rz = float(135)
        new_item_x = float(-200)
        new_item_y = float(400)
        new_item_rz = float(135)
        item_position = MotionCommand("move",{'X':item_x,'Y':item_y,'Rz':item_rz})
        item_grab = [MotionCommand("move",{'Z':item_z}),MotionCommand("grab",{}),MotionCommand("wait",{}),MotionCommand("move",{'Z':prong_safe_height})]
        new_item_position = MotionCommand("move",{'X':new_item_x,'Y':new_item_y,'Rz':new_item_rz,'Z':prong_safe_height})
        item_release = [MotionCommand("release",{}),MotionCommand("move",{'Z':prong_safe_height})]
        item_subroutine = [item_position]+item_grab+[new_item_position]+item_release+[MotionCommand("move",{'Z':prong_safe_height})]
        prong_motion.extend(item_subroutine)

    start = [MotionCommand("move_immutable",{'X':340,'Y':340,'Z':450,'Rx':180,'Ry':0,'Rz':135})]
    end = [MotionCommand("move_immutable",{'X':340,'Y':340,'Z':450,'Rx':180,'Ry':0,'Rz':135})]
    MCList = start + take_prong_motion + prong_motion + put_prong_motion + end

    print('Raw Motion Commands:')
    PrintRawMCList(MCList)
    print('\n\n=======================\n\n')
    m = Motion(MCList)
    print('Decoded Motion:')
    print(m)
    return m


def plan_cutting_motion():
    #start = [MotionCommand("move_immutable",{'X':340,'Y':340,'Z':450,'Rx':180,'Ry':0,'Rz':135})]

    #m = Motion(start)
    #return m

    timestamp = int(time.time())
    output_img_fn = 'images/image-%d.png' % timestamp

    cap = cv2.VideoCapture(0)
    ret,frame = cap.read()
    cv2.imwrite(output_img_fn, frame)

    cuts = get_cuts(frame, debug=True)


    blue_c, orange_c = get_blue_orange(frame, debug=False)

    fucking_camera_offset = np.array([8,22])

    stand_z = 172
    blue_x, blue_y = get_pixel_position(blue_c[0], blue_c[1], stand_z)
    orange_x, orange_y = get_pixel_position(orange_c[0], orange_c[1], stand_z)
    blue_xy = np.array([blue_x, blue_y])
    orange_xy = np.array([orange_x, orange_y])

    blue_xy += fucking_camera_offset
    orange_xy += fucking_camera_offset



    knife_xy = get_knife_xy(blue_xy, orange_xy)
    knife_xy_pre = get_knife_xy_pre(blue_xy, orange_xy)
    rz = get_rz(blue_xy, orange_xy)

    dist = np.linalg.norm(blue_xy-orange_xy)
    print(blue_x, blue_y)
    print(orange_x, orange_y)
    print(dist)

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





    knife_safe_height = {'Z':450}

    knife_cut_rot = {'Ry':-20}

    knife_rest_rot = {'Rx':180,'Ry':0, 'Rz':rz+90}
    knife_rest_xy = {'X':float(knife_xy[0]),'Y':float(knife_xy[1])}
    knife_rest_z = {'Z':171}
    knife_rest_z_2 = {'Z':170}
    knife_rest_pose = dict(knife_rest_z, **knife_rest_xy)
    knife_rest_pose_2 = dict(knife_rest_z_2, **knife_rest_xy)

    draw_knife_pose = {'X':float(knife_xy_pre[0]),'Y':float(knife_xy_pre[1]),'Z':200}
    draw_knife_xy = {'X':float(knife_xy_pre[0]),'Y':float(knife_xy_pre[1])}
    draw_knife_z = {'Z': 200}
    near_knife_z = {'Z': 200}
    z_offset = 30
    take_knife_motion = [\
        MotionCommand("move", knife_safe_height),
        MotionCommand("release", {}),
        MotionCommand("move", knife_rest_xy),
        MotionCommand("move", knife_rest_rot),
        MotionCommand("move", near_knife_z),
        MotionCommand("move", knife_rest_z),
        MotionCommand("grab", {}),
        MotionCommand("move", knife_rest_z_2),
        MotionCommand("move", knife_rest_z),
        MotionCommand("move", knife_rest_z_2),
        MotionCommand("move", knife_rest_z),
        MotionCommand("wait", {}),
        MotionCommand("move", draw_knife_pose),
        MotionCommand("move", knife_safe_height),
        MotionCommand("move", knife_cut_rot)]
    put_knife_motion = [\
        MotionCommand("move", knife_safe_height),
        MotionCommand("move", draw_knife_xy),
        MotionCommand("move", knife_rest_rot),
        MotionCommand("move", draw_knife_z),
        MotionCommand("move", knife_rest_pose),
        MotionCommand("release", {}),
        MotionCommand("move", knife_rest_pose_2),
        MotionCommand("move", knife_rest_pose),
        MotionCommand("move", knife_rest_pose_2),
        MotionCommand("move", knife_rest_pose),
        MotionCommand("move", knife_safe_height)]


    take_knife = take_knife_motion
    put_knife = put_knife_motion



    cut_position = [MotionCommand("move",{'X':200,'Y':450,'Z':450})]
    knife_subroutine = []
    knife_rotate = [MotionCommand("move", {'Ry':10})]


    cut_motion = []
    cut_safe_height = 350
    cut_z = 238
    cutting_board_z = 235
    cut_motion.append(MotionCommand("move", {'Z': cut_safe_height}))
    print('rz=', rz)
    for cut in cuts:
        cut_start_x, cut_start_y = get_pixel_position(cut[1][0], cut[1][1], cutting_board_z)
        cut_end_x, cut_end_y = get_pixel_position(cut[0][0], cut[0][1], cutting_board_z)
        cut_start_x = float(cut_start_x)
        cut_start_y = float(cut_start_y)
        cut_end_x = float(cut_end_x)
        cut_end_y = float(cut_end_y)
        cut_tmp_rz = float(np.rad2deg(np.arctan2(cut_end_y-cut_start_y,cut_end_x-cut_start_x)))
        if cut_tmp_rz < 70 and cut_tmp_rz > 30:
            cut_tmp_rz += 180
        if cut_tmp_rz < 0:
            cut_tmp_rz += 360

        knifetip_offset_x, knifetip_offset_y = get_knifetip_offset(cut_tmp_rz)
        print('cut_tmp_rz:', cut_tmp_rz)
        print('knifetip_offset:', knifetip_offset_x, knifetip_offset_y)
        cut_start_x += knifetip_offset_x
        cut_end_x += knifetip_offset_x
        cut_start_y += knifetip_offset_y
        cut_end_y += knifetip_offset_y
        cut_motion.append(MotionCommand("move", {'X': cut_start_x, 'Y':cut_start_y, 'Rz':cut_tmp_rz}))
        cut_motion.append(MotionCommand("move", {'Z': cut_z}))
        cut_motion.append(MotionCommand("move", {'X': cut_end_x, 'Y':cut_end_y, 'Rz':cut_tmp_rz}))
        cut_motion.append(MotionCommand("move", {'X': cut_start_x, 'Y':cut_start_y, 'Rz':cut_tmp_rz}))
        cut_motion.append(MotionCommand("move", {'Z': cut_safe_height}))
        print((cut_start_x, cut_start_y), (cut_end_x, cut_end_y), cut_tmp_rz)



    MCList = start + take_knife + cut_motion + put_knife + end


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
        elif MC.typename == 'wait':
            arm.wait()
        elif MC.typename == 'release':
            arm.release()
    return

def cut_proc(arm):
    "full procedure for cutting stuff"
    m = Motion([MotionCommand("away", {}), MotionCommand("release", {}), MotionCommand("wait", {})])
    execute_motion_dangerous(m, arm)
    m = plan_cutting_motion()
    execute_motion_dangerous(m, arm)


def main():
    # arm = Arm(X=340,Y=340,Z=450,Rx=180,Ry=0,Rz=135,gripper_open=False, use_killswitch=False)
    arm = Arm(X=340,Y=340,Z=450,Rx=180,Ry=0,Rz=135,gripper_open=False, use_killswitch=True)

    m = Motion([MotionCommand("away", {}), MotionCommand("release", {}), MotionCommand("wait", {})])
    execute_motion_dangerous(m, arm)
    m = plan_cutting_motion()
    execute_motion_dangerous(m, arm)

    # m = plan_prong_motion()
    # execute_motion_dangerous(m, arm)
    arm.stop_tm_driver()
    return


if __name__ == "__main__":
    main()
