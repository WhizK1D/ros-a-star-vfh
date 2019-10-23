#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from map_utils import *
from ros_pa2.msg import *


'''
Basic VFH that scans for obstacles using a laser scanner from
topic base_scan
'''


def vfhCallback(data):
    '''
    data here consists of 2 important parameters:
    ranges: distance of obstacles in each sample
    intensities: hit or miss in each sample
    Note: each sample refers to half a degree of alpha of histogram
    '''
    # Publish a cost for the obstacles found
    # Convert 2D ranges & intensities into 1D tuple for simpler calculations
    # Index can be used to compute the angle alpha of sample found
    laser_perception = tuple(l * r for l, r in zip(data.ranges, data.intensities))

    # Pick only the middle bin which is of interest when moving forward
    # Bin size ~= 36 degrees
    front_bin = laser_perception[120:240]

    # Internal values needed to calculate cost
    front_close_items = 0
    front_total_items = 0

    for f in front_bin:
        if (f > 0.0) and (f < 2.5):
            front_close_items += 1
        front_total_items += 1

    # Also check for empty areas on left and right
    left_bin = laser_perception[0:71]
    right_bin = laser_perception[288:361]

    left_far_items = 0
    left_total_items = 0
    right_far_items = 0
    right_total_items = 0

    for r in right_bin:
        if (r > 1.5):
            right_far_items += 1
        right_total_items += 1

    for l in left_bin:
        if (l > 1.5):
            left_far_items += 1
        left_total_items += 1

    # Note that front_cost function is only for avoiding obstacles
    # rather than choosing an "opening"
    front_cost = float(float(front_close_items)/front_total_items)
    rospy.loginfo("Current VFH cost function value: {}".format(str(front_cost)))
    warn_pub.publish(front_cost)

    # If left/right sides are free, choose one and publish free side
    left_cost = float(float(left_far_items)/left_total_items)
    right_cost = float(float(right_far_items)/right_total_items)

    if (max(left_cost, right_cost) == left_cost) and left_cost > 0.75:
        rospy.loginfo("[{}]LEFT side is risky!".format(str(left_cost)))
        free_pub.publish("LEFT")
    elif (max(left_cost, right_cost) == right_cost) and right_cost > 0.75:
        rospy.loginfo("[{}]RIGHT side is risky!".format(str(right_cost)))
        free_pub.publish("RIGHT")
    else:
        rospy.loginfo("Both sides are free")
        free_pub.publish("NONE")
    (rospy.Rate(10)).sleep()


def listener():
    rospy.init_node("VFH", anonymous=False)
    rospy.Subscriber("base_scan", LaserScan, vfhCallback)
    rospy.spin()


warn_pub = rospy.Publisher("vfh_cost", Float64, queue_size=10)
free_pub = rospy.Publisher("vfh_free", String, queue_size=10)


if __name__ == "__main__":
    listener()
