import numpy as np
import cv2

def csv_to_findhomography_input(csv_fn):
	import csv
	img_pts = []
	obj_pts = []
	with open(csv_fn, newline='') as f:
		reader = csv.DictReader(f)
		for row in reader:
			x_obj = float(row['x_obj'])
			y_obj = float(row['y_obj'])
			x_img = float(row['x_img'])
			y_img = float(row['y_img'])
			img_pts.append((x_img,y_img))
			obj_pts.append((x_obj,y_obj))
	obj_pts = np.array(obj_pts).astype('float32')
	img_pts = np.array(img_pts).astype('float32')
	return img_pts, obj_pts


def csv_to_solvepnp_input(csv_fn):
	import csv
	img_pts = []
	obj_pts = []
	with open(csv_fn, newline='') as f:
		reader = csv.DictReader(f)
		for row in reader:
			x_obj = float(row['x_obj'])
			y_obj = float(row['y_obj'])
			z_obj = float(row['z_obj'])
			x_img = float(row['x_img'])
			y_img = float(row['y_img'])
			img_pts.append((x_img,y_img))
			obj_pts.append((x_obj,y_obj,z_obj))
	obj_pts = np.array(obj_pts).astype('float32')
	img_pts = np.array(img_pts).astype('float32')
	return img_pts, obj_pts


csv_fn = 'mapping.csv'
img_pts, obj_pts = csv_to_findhomography_input(csv_fn)
M, mask = cv2.findHomography(img_pts, obj_pts) # src=img_pts, dst=obj_pts
print('Homography matrix:')
print(M)
_img_pts = np.transpose(np.array([[x[0],x[1],1.0] for x in img_pts]))

estimated_xy = np.matmul(M, _img_pts)
for i in range(estimated_xy.shape[1]):
        estimated_xy[0,i] /= estimated_xy[2,i]
        estimated_xy[1,i] /= estimated_xy[2,i]
        estimated_xy[2,i] = 150
estimated_xy = np.transpose(estimated_xy[:2,:].copy())
# estimated z = 150

print('real location:')
print(obj_pts)
print('estimated xy:')
print(estimated_xy)


#'''
mtx = np.array([[816.7299762,0,310.21960294],[0,818.1692256,248.89023889],[0,0,1]])
dist = np.array([[-0.00154756,0.46003915,0.00443635,-0.0032353,-1.2743261]])

img_pts, obj_pts = csv_to_solvepnp_input(csv_fn)
print(img_pts)
print(obj_pts)
_, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, mtx, dist)
#_, rvec, tvec, inliers = cv2.solvePnPRansac(obj_pts, img_pts, mtx, dist)
R, _ = cv2.Rodrigues(rvec)
#print(inliers)
print('Intrinsic matrix:')
print(mtx)
print('distortion coefficients')
print(dist)
print('Rotation matrix:')
print(R)
print('translation:')
print(tvec)

# project 3D points to image plane
img_pts_reprojected, _ = cv2.projectPoints(obj_pts, rvec, tvec, mtx, dist)
print(img_pts_reprojected)
#'''



