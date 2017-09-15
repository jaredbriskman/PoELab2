#!/usr/local/bin/python

from serial import Serial
import numpy as np
import matplotlib.pyplot as plt
from math import pi, cos, sin
import matplotlib as mpl
import matplotlib.colors as colors
import matplotlib.cm as cmx
from mpl_toolkits.mplot3d import Axes3D

# cxn = Serial('/dev/ttyACM0', baudrate=9600)
# while(True):
#     serial_val = cxn.readline()
#     serial_str = serial_val[:-2].decode("utf-8")
#     split_str = serial_str.split('x')


def get_euler(cxn):
    serial_val = cxn.readline()
    serial_str = serial_val[:-2].decode("utf-8")
    return serial_str.split('x')


def rotz(theta): # Generates a z-axis rotation matrix for a given angle
    return [[cos(theta), -sin(theta), 0],
            [sin(theta), cos(theta), 0],
            [0, 0, 1]]


def roty(phi): # Generates a y-axis rotation matrix for a given angle
    return [[1, 0, 0],
            [0, cos(phi), sin(phi)],
            [0, -sin(phi), cos(phi)]]


def get_origin(theta, phi):
    origin_init = [0, 0, 0] # Leaving this as 0 since the current pan/tilt design has no offset
    return np.dot(roty(phi), np.dot(rotz(theta), origin_init))


def transform_points(thetas, phis, lens):
    points = []
    points_euler = list(zip(thetas, phis, lens))
    points_cart = [point_transform(*point) for point in points_euler]
    print(points)
    return points_cart


def point_transform(theta, phi, dist):
    point_euler = [theta, phi, dist]
    origin_point = get_origin(*point_euler[:-1]) # account for movement of the origin of the sensor
    orig_vec = [0, point_euler[2], 0] # The original vector
    point_transform = np.dot(roty(point_euler[1]), np.dot(rotz(point_euler[0]), orig_vec))
    return point_transform+origin_point


# Constructs test list of points
theta_vals = np.linspace(-pi/2, pi/2, 10)
phi_vals = np.zeros(10)
len_vals = np.ones(10)
point_vals = transform_points(theta_vals, phi_vals, len_vals)
points_deconstruct = zip(*point_vals)

# pyplot setup
mpl.rcParams['toolbar'] = 'None'
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_axis_off()
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])
plt.ion()
autumn = plt.get_cmap('autumn')
cNorm = colors.Normalize(vmin=0, vmax=max(list(points_deconstruct)[1]))
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=autumn)

# Plot points

for point in point_vals:
    color_val = scalarMap.to_rgba(point[1])
    ax.scatter(*point, color=color_val)
    plt.show()
    plt.pause(0.05)
plt.show(block=True)
# cxn = Serial('/dev/ttyACM0', baudrate=9600)
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# plt.ion()
# while(True):
#     euler = get_euler(cxn)
#     point_cart = point_transform(*euler)
#     ax.scatter(point_cart)
#     plt.show()
#     plt.pause(.05)
