
import numpy as np
fx = 453.887961
fy = 461.229478
cx = 650.460817
cy = 330.965420

k1 = -0.162519
k2 = 0.015206
p1 = -0.004095
p2 = -0.001074
k3 = 0.000000

camera_matrix = np.array([[fx, 0, cx],
                          [ 0, fy, cy],
                          [0, 0, 1]])

distortion_coefficients = np.array([k1, k2, p1, p2, k3], dtype=np.float32)