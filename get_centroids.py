import cv2
import numpy as np
import os
import sys

img_dir = 'images'
img_fns = sorted([img_dir+'/'+x for x in os.listdir(img_dir) if x.endswith(".png") and x.startswith("img_")])
#print(img_fns)
output_csv_fn = "mapping.csv"

csv_str = "x_obj,y_obj,z_obj,x_img,y_img\n"

for img_fn in img_fns:
    img = cv2.imread(img_fn, cv2.IMREAD_GRAYSCALE)
    ret, th1 = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)
    num_labels, output, stats, centroids = cv2.connectedComponentsWithStats(th1, connectivity=8)
    #print(stats)
    good_label = [i for i in range(num_labels) if stats[i][4] >= 600 and stats[i][4] <= 3000][0]
    good_centroid = centroids[good_label]
    #print(good_label, good_centroid)

    xy = os.path.splitext(os.path.basename(img_fn))[0]
    _,x,y = xy.split('_')

    line = "%s,%s,%d,%f,%f\n" % (x,y,153,good_centroid[0],good_centroid[1])
    csv_str += line

    #cv2.imshow('img', th1)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

print(csv_str)
with open(output_csv_fn, 'w') as f:
    f.write(csv_str)
print('saving output to %s' % output_csv_fn)
