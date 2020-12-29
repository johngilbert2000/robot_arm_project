import cv2
import numpy as np

def get_circles(img, color_name, debug=False):
    colors = {}
    colors['orange'] = [np.array([11,43,46]), np.array([25,255,255])]
    colors['blue'] = [np.array([80,43,46]), np.array([124,255,255])]
    colors['green'] = [np.array([35,43,46]), np.array([77,255,255])]
    color = colors[color_name]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, color[0], color[1])
    if debug:
        cv2.imshow('img', img)
        cv2.waitKey(0)
        cv2.imshow('mask', mask)
        cv2.waitKey(0)
    kernel = np.ones((4,4), np.uint8)
    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if debug:
        print(stats)
    min_comp = 200
    max_comp = 500
    good_labels = [i for i in range(num_labels) if stats[i][4]<max_comp and stats[i][4]>min_comp]
    ret = centroids[good_labels]
    return ret


def get_red_circles(img, debug=False):
    colors = []
    red1 = [np.array([0,150,46]), np.array([5,255,255])]
    red2 = [np.array([0,150,46]), np.array([5,255,255])]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, red1[0], red1[1])
    mask2 = cv2.inRange(hsv, red2[0], red2[1])
    mask = cv2.bitwise_or(mask1, mask2)
    if debug:
        cv2.imshow('img', img)
        cv2.waitKey(0)
        cv2.imshow('mask', mask)
        cv2.waitKey(0)
    kernel = np.ones((4,4), np.uint8)
    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if debug:
        print(stats)
    min_comp = 200
    max_comp = 500
    good_labels = [i for i in range(num_labels) if stats[i][4]<max_comp and stats[i][4]>min_comp]
    ret = centroids[good_labels]


def get_blue_orange(img, debug=False):
    blue_circles = get_circles(img, 'blue', debug=debug)
    orange_circles = get_circles(img, 'orange', debug=debug)
    blue_c = blue_circles[0]
    dist_to_blue_circles = [np.linalg.norm(x-blue_c) for x in orange_circles]
    orange_c = orange_circles[np.argmin(dist_to_blue_circles)]
    return blue_c, orange_c


def get_food_region(img, debug=False):
    blue_c, orange_c = get_blue_orange(img, False)
    unit_x = orange_c - blue_c
    unit_x /= np.linalg.norm(unit_x)
    unit_y = np.array([-unit_x[1], unit_x[0]])
    mask = np.ones(frame.shape[:2])
    origin = blue_c

    a = np.stack([unit_x, unit_y], axis=1)

    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            b = np.array([origin[0]-i, j-origin[1]])
            x = np.linalg.solve(a, b)
            if x[0] > -20 and x[0] < 130 and x[1] > -70 and x[1] < 260:
                mask[i,j] = 0

    colors = []
    red1 = [np.array([0,43,46]), np.array([10,255,255])]
    red2 = [np.array([160,43,46]), np.array([180,255,255])]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, red1[0], red1[1])
    mask2 = cv2.inRange(hsv, red2[0], red2[1])
    maskred = cv2.bitwise_or(mask1, mask2)
    maskred = cv2.bitwise_not(maskred)

    #mask = cv2.bitwise_or(mask, maskred)
    #print(maskred.shape)
    #print(mask.shape)

    for i in range(maskred.shape[0]):
        for j in range(maskred.shape[1]):
                if maskred[i,j] == 0:
                    mask[i,j] = 0

    if debug:
        cv2.imshow('img', img)
        cv2.imshow('mask', mask)
        cv2.imshow('maskred', maskred)
        cv2.waitKey(0)
    mask = np.stack([mask for i in range(3)], axis=2).astype(np.uint8)

    return mask


def get_cutting_board_mask(img, debug=False):
    colors = []
    red1 = [np.array([0,43,46]), np.array([10,255,255])]
    red2 = [np.array([160,43,46]), np.array([180,255,255])]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, red1[0], red1[1])
    mask2 = cv2.inRange(hsv, red2[0], red2[1])
    mask = cv2.bitwise_or(mask1, mask2)
    #mask = cv2.bitwise_not(mask)
    mask = cv2.erode(mask, np.ones((5,5), np.uint8))
    mask = cv2.dilate(mask, np.ones((5,5), np.uint8))
    mask = cv2.dilate(mask, np.ones((20,20), np.uint8))
    mask = cv2.dilate(mask, np.ones((20,20), np.uint8))
    mask = cv2.erode(mask, np.ones((20,20), np.uint8))
    mask = cv2.erode(mask, np.ones((20,20), np.uint8))


    # Find contours
    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[1]
    print([i.shape for i in contours])
    contours = np.concatenate(contours, axis=0)
    print(contours.shape)
    # Find the convex hull object for each contour
    hull = cv2.convexHull(contours)
    print(hull.shape)
    hull = hull.reshape([hull.shape[0], 2])

    plot_hull = np.zeros_like(img)
    for i in range(len(hull)):
        cv2.circle(plot_hull,(hull[i][0], hull[i][1]), 15, (255, 0, 0), -1)
    cv2.imshow('plot_hull', plot_hull)
    cv2.waitKey(0)



    if debug:
        cv2.imshow('img', img)
        cv2.imshow('mask', mask)
        cv2.waitKey(0)

def get_food_only_photo(img, debug=False):
    mask = get_food_region(img, debug)
    ret = img.copy()
    for i in range(frame.shape[0]):
        for j in range(frame.shape[1]):
            for k in range(frame.shape[2]):
                if mask[i,j,k] == 0:
                    ret[i,j,k] = 0
    cv2.imshow('food_only', ret)
    cv2.waitKey(0)
    return ret

def find_food(img, debug=False):
    food_only_photo = get_food_only_photo(frame, debug)
    colors = {}
    colors['white'] = [np.array([200,200,200]), np.array([255,255,255])]
    color = colors['white']
    mask = cv2.inRange(img, color[0], color[1])
    return



if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    ret,frame = cap.read()
    cap.release()

    #get_red_circles(frame, True)
    #food = find_food(frame, True)

    get_cutting_board_mask(frame, True)
    exit(0)

    blue_c, orange_c = get_blue_orange(frame, True)
    print('blue:')
    print(blue_c)
    print('orange:')
    print(orange_c)


