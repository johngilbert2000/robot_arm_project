def get_pixel_position(img_x, img_y, robot_z=150):
	import numpy as np
	import cv2
	# intrinsic matrix
	K = np.array([[816.7299762,0.0,310.21960294],
	[0.0,818.1692256,248.89023889],
	[0.0,0.0,1.0]])
	# distortion coefficients
	dist = np.array([[-0.00154756,0.46003915,0.00443635,-0.0032353,-1.2743261]])
	# rotation matrix
	R = np.array([[-0.69773536,0.71632187,-0.00695332],
	[0.7156508,0.69658555,-0.05111259],
	[-0.03176949,-0.04063921,-0.99866869]])
	# translation
	T = np.array([[7.76329826],[-462.74499577],[977.88132645]])
	# undistort
	pt_2d = np.array([img_x, img_y]).astype('float32').reshape(1,1,2)
	pt_2d = cv2.undistortPoints(pt_2d,K,dist,None,K).reshape(2)
	img_x, img_y = pt_2d
	RT_at_current_height = R.copy()
	RT_at_current_height[:,2] *= robot_z
	for i in range(3):
		RT_at_current_height[i,2] += T[i]
	KRT_at_current_height = np.matmul(K, RT_at_current_height)
	H_at_current_height = np.linalg.inv(KRT_at_current_height)
	H_at_current_height /= H_at_current_height[2,2]
	v = np.array([img_x, img_y, 1.0], dtype=np.float32)
	Hv = np.matmul(H_at_current_height,v)
	robot_x, robot_y = Hv[0]/Hv[2], Hv[1]/Hv[2]
	return float(robot_x), float(robot_y)

def old_get_pixel_position(img_x, img_y):
	import numpy as np
	# homography
	H = np.array([[-6.93065402e-01,7.39083356e-01,3.79221573e+02],
		 [ 7.12310652e-01,7.20020552e-01,-7.03669953e+01],
	 [ 2.89548798e-07,7.35111617e-05,1.00000000e+00]])
	v = np.array([img_x, img_y, 1.0], dtype=np.float32)
	Hv = np.matmul(H,v)
	robot_x, robot_y = Hv[0]/Hv[2], Hv[1]/Hv[2]
	return robot_x, robot_y

if __name__ == "__main__":
	print(old_get_pixel_position(80.44, 380.53))
	print(get_pixel_position(80.44, 380.53, robot_z=153))
	# print(get_pixel_position(611.0, 197.0, robot_z=220))



