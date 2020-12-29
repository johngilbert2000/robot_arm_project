def img_xy_to_z_150_xy(img_x, img_y):
    import numpy as np
    H = np.array([[-6.93065402e-01,7.39083356e-01,3.79221573e+02],
         [ 7.12310652e-01,7.20020552e-01,-7.03669953e+01],
     [ 2.89548798e-07,7.35111617e-05,1.00000000e+00]])
    #print(H)
    v = np.array([img_x, img_y, 1.0], dtype=np.float32)
    Hv = np.matmul(H,v)
    return Hv[0]/Hv[2], Hv[1]/Hv[2]

#x,y = img_xy_to_z_150_xy(611.789,197.809)
#print(x,y)
