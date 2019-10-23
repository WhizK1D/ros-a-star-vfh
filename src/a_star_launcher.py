#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from map_utils import *
from a_star_utils import *
from ros_pa2.msg import *
from robot import *


if __name__ == "__main__":
    rospy.init_node("a_star_launcher", anonymous=False)

    map = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
            0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
            0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
            0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
            0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
            0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
            0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
            0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
            0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
            0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

    goalx = rospy.get_param("goalx")
    goaly = rospy.get_param("goaly")

    rospy.loginfo("Found goal in ROS param: {}, {}".format(str(goalx), str(goaly)))

    source_position = rospy.wait_for_message("base_pose_ground_truth",
                        Odometry).pose.pose.position

    goal_cell = inGridCell(goalx, goaly)
    start_cell = inGridCell(source_position.x, source_position.y)

    rospy.loginfo("Goal: {} Start: {}".format(str(goal_cell), str(start_cell)))
    grid = Grid(map)

    sc = start_cell[0]
    sr = start_cell[1]

    gc = goal_cell[0]
    gr = goal_cell[1]

    path = grid.A_star(s=grid.grid[sr][sc], g=grid.grid[gr][gc])
    if path is None:
        exit()

    print "Map and path visualization:"
    # Cheap visualizer trick to check path returned by A_star()
    for i in range(len(grid.grid)):
        for j in range(len(grid.grid[0])):
            if (grid.grid[i][j]).obstacle == True:
                print bcolors.FAIL + "1" + bcolors.ENDC,
            elif (grid.grid[i][j]) in path:
                print bcolors.OKGREEN + "+" + bcolors.ENDC,
            else:
                print bcolors.OKBLUE + "0" + bcolors.ENDC,
        print ""

    for node in path:
        print node

    a_star_robo = Robot(grid.grid[gr][gc])

    VFH_THRESHOLD = 0.35
    temp_path = path[:]
    next_node = temp_path.pop(0)
    while a_star_robo.state != states.TARGET_REACHED:
        '''
        1. Take path from A_star() output
        2. For each node, traverse to node using move_towards_target() while
            cost from VFH < VFH_THRESHOLD
            2a. If obstacle, read vfh_free to rotate until path free
        3. If still in path, move to next node
            3a. Else go to step 1
        '''
        robo_state = rospy.wait_for_message("robot_data", RobotData).state
        while robo_state != "OBSTACLE":
            rospy.loginfo("Moving to node: {} {}".format(str(next_node.row), str(next_node.col)))
            a_star_robo.move_to_cell(next_node)
            if len(temp_path) > 0:
                next_node = temp_path.pop(0)
            else:
                next_node = path.pop()
                a_star_robo.move_to_cell(next_node)
                rospy.loginfo("Probably reached goal!")
                exit()
            robo_state = rospy.wait_for_message("robot_data", RobotData).state

        if robo_state == "OBSTACLE":
            a_star_robo.avert_obstacle()
            cur_node = a_star_robo.get_current_cell()
            cur_cell = grid.grid[cur_node.row][cur_node.column]
            if cur_cell in path:
                a_star_robo.state = states.SAFE
                continue