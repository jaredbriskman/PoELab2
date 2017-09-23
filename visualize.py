#!/usr/local/bin/python

from serial import Serial
import numpy as np
import matplotlib.pyplot as plt
from math import pi, cos, sin
import matplotlib as mpl
import matplotlib.colors as colors
import matplotlib.cm as cmx
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import csv

cxn = Serial('/dev/ttyACM0', baudrate=38400)
# while(True):
#     serial_val = cxn.readline()
#     serial_str = serial_val[:-2].decode("utf-8")
#     split_str = serial_str.split('x')


def get_euler(cxn):
    serial_val = cxn.readline()
    if len(serial_val) >= 8:
        try:
            serial_str = serial_val[:-2].decode("utf-8")
        except:
            pass
        str_split = serial_str.split('x')
        if '' not in str_split:
            return str_split


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
    point_euler = [int(theta)*(pi/180), int(phi)*(pi/180), int(dist)/100]
    origin_point = get_origin(*point_euler[:-1]) # account for movement of the origin of the sensor
    orig_vec = [0, point_euler[2], 0] # The original vector
    point_transform = np.dot(roty(point_euler[1]), np.dot(rotz(point_euler[0]), orig_vec))
    return point_transform+origin_point



# Plot points

# for point in point_vals:
#     color_val = scalarMap.to_rgba(point[1])
#     ax.scatter(*point, color=color_val)
#     plt.show()
#     plt.pause(0.05)
# plt.show(block=True)
# cxn = Serial('/dev/ttyACM0', baudrate=9600)
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# plt.ion()

# all_points = []
# while(len(all_points) < 60*90):
#     if cxn.inWaiting:
#         euler = get_euler(cxn)
#         if euler:
#             if '$' in ''.join(euler):
#                 break
#             all_points.append(euler)
            # point_cart = point_transform(*euler)
            # color_val = scalarMap.to_rgba(point_cart[1])

# print(all_points)
# df = pd.DataFrame(list(all_points), columns=list('xyz'))
# print(df)
# df.to_csv('points.txt')

# Constructs test list of points
with open('points.txt', 'r') as points_file:
    points_reader = csv.reader(points_file)
    list_points = list(points_reader)[1:]
    points_clean = [point[1:] for point in list_points]

points_split = zip(*points_clean)
point_vals = transform_points(*points_split)
points_deconstruct = zip(*point_vals)

# pyplot setup
mpl.rcParams['toolbar'] = 'None'
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_axis_off()
ax.scatter(*points_deconstruct)
plt.show()
# # ax.set_xlim([-1, 1])
# # ax.set_ylim([-1, 1])
# # ax.set_zlim([-1, 1])
# plt.ion()
# autumn = plt.get_cmap('autumn')
# cNorm = colors.Normalize(vmin=0, vmax=1)
# scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=autumn)
