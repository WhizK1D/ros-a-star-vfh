#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float64
from ros_pa2.msg import RobotData
from ros_pa2.msg import MapLocation
from ros_pa2.msg import GridLocation
from map_utils import *


def locationCallback(data):
    robo_loc = data.pose.pose.position
    robo_orientation = data.pose.pose.orientation
    map_x = robo_loc.x
    map_y = robo_loc.y

    grid_x = mapToGrid(robo_loc.x, robo_loc.y)[0]
    grid_y = mapToGrid(robo_loc.x, robo_loc.y)[1]

    grid_row = inGridCell(map_x, map_y)[1]
    grid_col = inGridCell(map_x, map_y)[0]

    robo_data = RobotData()
    grid_location = GridLocation()
    map_location = MapLocation()

    grid_location.row = grid_row
    grid_location.column = grid_col

    map_location.x = map_x
    map_location.y = map_y

    rospy.loginfo("Tatti")

    robo_data.gridLocation = grid_location
    robo_data.mapLocation = map_location

    robo_state = "GLOBAL_PLANNING"
    if (grid_col == goal_cell[0]) and (grid_row == goal_cell[1]):
        robo_state = "TARGET_REACHED"

    robo_data_publisher.publish(robo_data)
    vfh_cost = (rospy.wait_for_message("vfh_cost", Float64, 0.2)).data

    if vfh_cost < 0.92 and robo_state != "TARGET_REACHED":
        robo_state = "SAFE"
    elif vfh_cost > 0.92:
        robo_state = "OBSTACLE"
        rospy.logwarn("Obstacle found")
    robo_data.state = robo_state
    str_gr = str(grid_location.row)
    str_gc = str(grid_location.column)
    rospy.loginfo("{} {};".format(str_gr, str_gc))

    rospy.loginfo("Tatti2")

    rospy.loginfo("{} {}; Current robo state: {}; VFH cost: {}".format(str_gr, str_gc, robo_state, str(vfh_cost)))
    # (rospy.Rate(10)).sleep()


def listener():
    rospy.Subscriber("base_pose_ground_truth", Odometry, locationCallback)
    rospy.spin()


rospy.init_node("Overwatch", anonymous=False)
robo_data_publisher = rospy.Publisher("robot_data", RobotData, queue_size=10)
goalx = rospy.get_param("goalx")
goaly = rospy.get_param("goaly")
goal_cell = inGridCell(goalx, goaly)


if __name__ == "__main__":
    listener()
