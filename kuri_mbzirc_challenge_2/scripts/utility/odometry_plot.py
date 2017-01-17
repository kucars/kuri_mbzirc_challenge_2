#!/usr/bin/env python

import rosbag

import datetime
from tf.msg import tfMessage
from argparse import ArgumentParser
from geometry_msgs.msg import Quaternion
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix
from tf.transformations import euler_from_quaternion,quaternion_from_euler,quaternion_multiply,quaternion_matrix
import tf

from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D

# Prompt user if scipy is missing.
try:
  from scipy import optimize
except ImportError:
  print("This script requires scipy be available.")
  print("On Ubuntu: sudo apt-get install python-scipy")
  exit(1)

parser = ArgumentParser(description='Plot GPS and odometry data.')
parser.add_argument('bag', metavar='FILE', type=str, help='input bag file')
args = parser.parse_args()

bag = rosbag.Bag(args.bag)

# Read odometry/filtered points
odometry_tuples_x = []
odometry_tuples_y = []
odometry_tuples_z = []
for topic, msg, time in bag.read_messages(topics=("/odometry/filtered", "/odometry/filtered")):
        odometry_tuples_x.append(msg.pose.pose.position.x)
        odometry_tuples_y.append(msg.pose.pose.position.y)
        odometry_tuples_z.append(msg.pose.pose.position.z)

# Read odometry/gps points
gps_tuples_x = []
gps_tuples_y = []
gps_tuples_z = []
for topic, msg, time in bag.read_messages(topics=("/odometry/gps", "/odometry/gps")):
        gps_tuples_x.append(msg.pose.pose.position.x)
        gps_tuples_y.append(msg.pose.pose.position.y)
        gps_tuples_z.append(msg.pose.pose.position.z)

# Plot
print( "Found " + str(len(odometry_tuples_x)) + " points in /odometry/filtered" )
print( "Found " + str(len(gps_tuples_x)) + " points in /odometry/gps" )

fig = pyplot.figure()
ax1 = fig.add_subplot(211)
ax1.scatter(odometry_tuples_x, odometry_tuples_y)
ax1.set_aspect('equal', adjustable='box')
ax2 = fig.add_subplot(212)
ax2.scatter(gps_tuples_x, gps_tuples_y)
ax2.set_aspect('equal', adjustable='box')
pyplot.show()

exit(1)
