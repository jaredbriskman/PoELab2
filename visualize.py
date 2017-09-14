#!/usr/local/bin/python

from serial import Serial, SerialException
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import pi, cos, sin

# cxn = Serial('/dev/ttyACM0', baudrate=9600)
# cur_value = ''
# while(True):
#     serial_val = cxn.readline()
#     serial_str = serial_val[:-2].decode("utf-8")
#     split_str = serial_str.split('x')
def rotz(theta):
    return [[cos(theta), -sin(theta), 0],
            [sin(theta), cos(theta), 0],
            [0, 0, 1]]


def roty(phi):
    return [[1, 0, 0],
            [0, cos(phi), sin(phi)],
            [0, -sin(phi), cos(phi)]]


def get_origin(theta, phi):
    origin_init = [0, 0, 1]
    return np.dot(roty(phi), np.dot(rotz(theta), origin_init))


def transform_points(thetas, phis, lens):
    points = []
    points_euler = list(zip(thetas, phis, lens))
    for point in points_euler:
        origin_point = get_origin(*point[:-1]) # account for movement of the origin of the sensor
        orig_vec = [0, point[2], 0] # The original vector
        point_transform = np.dot(roty(point[1]), np.dot(rotz(point[0]), orig_vec))
        points.append(point_transform+origin_point)
    print(points)
    return points

theta_vals = np.linspace(-pi/2, pi/2, 10)
phi_vals = np.zeros(10)
len_vals = np.ones(10)
point_vals = transform_points(theta_vals, phi_vals, len_vals)
points_deconstruct = zip(*point_vals)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(*points_deconstruct)
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])

# rot_vec = np.dot(roty, np.dot(rotz, init_vec))
# ax.quiver(0, 0, 0, *rot_vec)
plt.show()
