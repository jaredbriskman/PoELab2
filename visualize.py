"""
PoE Lab 2 Visualization Code
Jared Briskman and Matt Brucker
"""
#!/usr/local/bin/python

from serial import Serial
import numpy as np
import matplotlib.pyplot as plt
from math import pi, cos, sin
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import csv

cxn = Serial('/dev/ttyACM0', baudrate=9600)


# Reads a point in spherical form via serial data.
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


def rotz(theta):  # Generates a z-axis rotation matrix for a given angle
    return [[cos(theta), -sin(theta), 0],
            [sin(theta), cos(theta), 0],
            [0, 0, 1]]


def roty(phi):  # Generates a y-axis rotation matrix for a given angle
    return [[1, 0, 0],
            [0, cos(phi), sin(phi)],
            [0, -sin(phi), cos(phi)]]


# Transforms a list of points from spherical coordinates to Cartesian
def transform_points(thetas, phis, lens):
    points = []
    points_euler = list(zip(thetas, phis, lens))
    points_cart = [point_transform(*point) for point in points_euler]
    print(points)
    return points_cart


def point_transform(theta, phi, dist):
    point_euler = [(int(theta)-100)*(pi/180), -(int(phi)-90)*(pi/180), int(dist)/100]
    orig_vec = [0, point_euler[2], 0]  # The original vector to transform
    point_transform = np.dot(roty(point_euler[1]), np.dot(rotz(point_euler[0]), orig_vec))
    return point_transform


def points_to_csv(file_name='points.txt'):
    all_points = []
    while(len(all_points) < 60*90):  # While we've received less than the maximum number of points
        if cxn.inWaiting:  # Wait to receive new serial data
            euler = get_euler(cxn)
            if euler:  # If we get valid serial data:
                if '$' in ''.join(euler):  # Stop if we reach the end
                    break
                all_points.append(euler)
    df = pd.DataFrame(list(all_points), columns=list('xyz'))
    df.to_csv(file_name)
    return all_points


# Sets up pyplot and returns the figure
def get_pyplot():
    mpl.rcParams['toolbar'] = 'None'
    plt.rcParams['image.cmap'] = 'gist_stern'
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_axis_off()
    return ax


points_to_csv()
# Open the list of points and format them correctly
with open('points.txt', 'r') as points_file:
    points_reader = csv.reader(points_file)
    list_points = list(points_reader)[1:]
    points_clean = [point[1:] for point in list_points]

# Converts the list of points from spherical to Cartesian coordinates
points_split = zip(*points_clean)
point_vals = transform_points(*points_split)
points_deconstruct = zip(*point_vals)
points_deconstruct2 = zip(*point_vals) # Needs a copy for the color map to work

# Plots the points and generates the heatmap
ax = get_pyplot()
ax.scatter(*points_deconstruct, c=list(points_deconstruct2)[1])
plt.show()
