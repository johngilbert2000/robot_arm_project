#!/usr/bin/env python3

import math
import cv2
import numpy as np
from get_pixel_position import get_pixel_position
from arm_class import Arm
import time

def find_empty_spot(X, Y, padding=10, stride=5):
    maxMargin = 0
    maxMarginX = 0
    maxMarginY = 0
    sqrt2 = math.sqrt(2)
    # (x,y) = 1/sqrt(2) * (u(1,1) + v*(-1,1))
    for u in range(0+padding, 200-padding, stride):
        for v in range(-200+padding, 200-padding, stride):
            x = 200 + (u - v)
            y = 200 + (u + v)
            margin = 1e9
            for i in range(len(X)):
                r = math.sqrt((X[i]-x)**2 + (Y[i]-y)**2)
                margin = min(margin,r)
            if margin > maxMargin:
                maxMargin = margin
                maxMarginX = max(maxMarginX, x)
                maxMarginY = max(maxMarginY, y)
    return maxMarginX, maxMarginY

def find_empty_spot_by_image(img, pause=False, verbose=True):
    ret, thresh = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY)
    if pause:
        cv2.imshow('img', thresh)
        cv2.waitKey(0)

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresh, connectivity=8)
    good_labels = [i for i in range(num_labels) if stats[i][4] > 600 and stats[i][4] <= 2000]
    good_centroids = centroids[good_labels]
    if verbose:
        print(good_centroids)

    maxMargin = 0
    maxMarginX = 0
    maxMarginY = 0
    sqrt2 = math.sqrt(2)
    padding = 100
    stride = 5
    X_END = 640
    Y_END = 400
    for x in range(padding, X_END-padding, stride):
        for y in range(padding, Y_END-padding, stride):
            margin = 1e9
            for i in range(len(good_centroids)):
                r = math.sqrt((good_centroids[i][0]-x)**2 + (good_centroids[i][1]-y)**2)
                margin = min(margin,r)
            if margin > maxMargin:
                maxMargin = margin
                maxMarginX = max(maxMarginX, x)
                maxMarginY = max(maxMarginY, y)
    return maxMarginX, maxMarginY


def main():
    # arm = Arm(gripper_open=False)
    arm = Arm()
    arm.use_killswitch = False
    #arm.center()
    arm.away()
    time.sleep(10)

    cam = cv2.VideoCapture(0)
    ret, image_colored = cam.read()
    cam.release()

    image = cv2.cvtColor(image_colored, cv2.COLOR_BGR2GRAY)
    #image = cv2.imread('opencv_frame_1.png', cv2.IMREAD_GRAYSCALE)

    #cv2.imshow('img',image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    xspot_image, yspot_image = find_empty_spot_by_image(image)
    xspot, yspot = [float(i) for i in get_pixel_position(xspot_image, yspot_image)]

    mask = np.zeros_like(image,dtype='int')
    mask[50:500,50:550] = 1
    workImage = np.array(image * mask).astype('uint8')

    #cv2.imshow('workImage',workImage)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    workImage = cv2.GaussianBlur(workImage, (3,3), 0)
    ret, thresh = cv2.threshold(workImage, 180, 255, cv2.THRESH_BINARY)

    workImage = cv2.erode(thresh, (7,7), iterations=1)
    workImage = cv2.dilate(workImage, (7,7), iterations=1)

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(workImage, connectivity=8)

    contours, hierarchy = cv2.findContours(workImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(workImage, contours, -1, (0,255,0), 3)
    cut = []
    for j in range(len(stats)):
        if stats[j][4] < 600 or stats[j][4] > 10000:
            cut.append(j)


    xs = []
    ys = []
    angles = []

    base_height = 155

    for i in contours:
        m = cv2.moments(i)
        cx = m['m10']/(m['m00'] + 1e-5)
        cy = m['m01']/(m['m00'] + 1e-5)



        skip = False
        for j in cut:
            if (cx-centroids[j][0])**2+(cy-centroids[j][1])**2 < 100:
                skip = True
                break
        if skip or cx==0 or cy==0:
            continue
        pa = 90 - math.atan2(2 * m['mu11'], m['mu20'] - m['mu02']) / 2 / math.pi *180
        print("centroid_x(" + str(cx) + ") centroid_y(" + str(cy) + ") principle_angle(" + str(pa) + ")")

        robot_x, robot_y = [float(i) for i in get_pixel_position(cx, cy)]
        robot_angle = float(pa) - 45
        if robot_angle <= 0:
            robot_angle += 180
        elif robot_angle >= 360:
            robot_angle -= 180

        xs.append(robot_x)
        ys.append(robot_y)
        angles.append(robot_angle)


    # xspot, yspot = find_empty_spot(xs, ys, padding=50)
    print(f"Spot -- x: {xspot:.2f} Y: {yspot:.2f}\n")


    for robot_x, robot_y, robot_angle in zip(xs, ys, angles):
        print('Robot moving to x=%f,y=%f' % (robot_x, robot_y))
        arm.move(X=robot_x, Y=robot_y, Rz=robot_angle)
        arm.move_down()
        arm.grab()
        arm.move_up()
        arm.move(X=xspot, Y=yspot, Rz=135)
        arm.move_down(base_height)
        arm.release()
        arm.move_up()
        base_height += 20
        time.sleep(10)



if __name__ == "__main__":
    main()


