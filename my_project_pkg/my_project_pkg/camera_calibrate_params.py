
import numpy as np
fx = 225.132714
fy = 227.862349
cx = 307.094696
cy = 192.895543

k1 = -0.191689
k2 = 0.022802
p1 = 0.003392
p2 = -0.001393
k3 = 0.000000

camera_matrix = np.array([[fx, 0, cx],
                          [ 0, fy, cy],
                          [0, 0, 1]])

distortion_coefficients = np.array([k1, k2, p1, p2, k3], dtype=np.float32)