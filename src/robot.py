import math
from enum import Enum
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from ros_pa2.msg import RobotData
from map_utils import *
import numpy

'''
Global enum that defines the possible states of the robot
'''
states = Enum("States",
        "GLOBAL_PLANNING SAFE OBSTACLE TARGET_REACHED FAILED")


class Robot:
    '''
    Robot base class to include basic initializations & utilities
    '''

    def __approx_equals(self, a, b, tolerance=0.01):
        '''
        Compare 2 values a & b with a default tolerance of 0.01.
        Tells if a is approximately equal to b.
        '''
        return (abs(a-b) <= (0.5 * tolerance * (a + b)))

    def __get_path_slope(self, point1, point2):
        '''
        Slope of straight path from point1 to point2 on map
        '''
        x1 = point1[0]
        y1 = point1[1]
        x2 = point2[0]
        y2 = point2[1]
        if (x2 < 0) and (x1 < 0):
            rospy.loginfo("both similarly negative")
            condition = self.__approx_equals(abs(x2), abs(x1), tolerance=0.1)
        else:
            rospy.loginfo("both similarly positive")
            condition = self.__approx_equals(x2, x1, tolerance=0.1)
        if condition:
            rospy.loginfo("Hit condition")
            if y2 > y1:
                return (float("inf"))
            else:
                return (-float("inf"))
        return (float(y2 - y1)/(x2 - x1))

    def move(self, lin, ang):
        '''
        Generic method to publish movement messages
        '''
        move_msg = Twist()
        move_msg.linear.x = lin
        move_msg.angular.z = ang
        self.vel_pub.publish(move_msg)

    def get_current_position(self):
        '''
        Get the position of the robot on the map as mapLocation
        '''
        return (rospy.wait_for_message("robot_data", RobotData).mapLocation)

    def get_current_cell(self):
        '''
        Get the position of the robot on the grid as gridLocation
        '''
        return (rospy.wait_for_message("robot_data", RobotData).gridLocation)

    def get_current_orientation(self):
        '''
        Get the current orientation of the robot by subscribing to the topic
        /base_pose_ground_truth of type nav_msgs/Odometry.pose.orientation
        '''
        return (rospy.wait_for_message("base_pose_ground_truth",
                Odometry).pose.pose.orientation)

    @DeprecationWarning
    def align_towards_target(self):
        '''
        Rotating the robot towards the target (to be used only when on m-line)
        '''
        turn_msg = Twist()
        # Align the robot towards the target
        curr_orientation = self.get_current_orientation()
        # temp is the orientation of robot in radians
        temp = ((tf.transformations.euler_from_quaternion((
            curr_orientation.x,
            curr_orientation.y,
            curr_orientation.z,
            curr_orientation.w
            )))[2])
        while not self.__approx_equals(math.atan(self.mline_slope), temp):
            turn_msg.angular.z = 0.05
            self.vel_pub.publish(turn_msg)
            curr_orientation = self.get_current_orientation()
            temp = ((tf.transformations.euler_from_quaternion((
                curr_orientation.x,
                curr_orientation.y,
                curr_orientation.z,
                curr_orientation.w
                    )))[2])
        rospy.loginfo("Done aligning towards target")

    def align_towards_target(self, target):
        '''
        Rotating robot towards specified target
        target - tuple of (x, y)
        '''
        rospy.loginfo("Beginning to align")
        curr_orientation = self.get_current_orientation()

        cur_radians = ((tf.transformations.euler_from_quaternion((
            curr_orientation.x,
            curr_orientation.y,
            curr_orientation.z,
            curr_orientation.w
            )))[2])

        cur_position = self.get_current_position()
        cur_point = (cur_position.x, cur_position.y)
        rospy.loginfo("{}".format(str(self.__get_path_slope(cur_point, target))))
        temp = math.atan(self.__get_path_slope(cur_point, target))
        temp2 = temp
        rospy.loginfo("Pos: {} {}; Target: {} {}".format(str(cur_point[0]), str(cur_point[1]), str(target[0]), str(target[1])))
        rospy.loginfo("Temp: {}; Cur: {}".format(str(temp), str(cur_radians)))

        if temp > cur_radians:
            turn_factor = 0.02
        else:
            turn_factor = -0.02
        while not self.__approx_equals(temp, cur_radians):
            temp = temp2
            curr_orientation = self.get_current_orientation()
            cur_radians = ((tf.transformations.euler_from_quaternion((
                curr_orientation.x,
                curr_orientation.y,
                curr_orientation.z,
                curr_orientation.w
                )))[2])
            rospy.loginfo("Temp: {}; Cur: {}".format(str(temp), str(cur_radians)))
            self.move(0, turn_factor)
            if (temp < 0) and (cur_radians < 0):
                temp = abs(temp)
                cur_radians = abs(cur_radians)
        rospy.loginfo("Alignment Complete!")

    def move_to_cell(self, cell):
        '''
        Depending on current location, align and move towards cell
        '''
        target = gridCenter(cell.row, cell.col)
        curr_cell = self.get_current_cell()

        if (curr_cell.row == cell.row) and (curr_cell.column == cell.col):
            rospy.loginfo("Already in cell, reached!")
            return
        self.move_towards_target(target)

    def avert_obstacle(self):
        '''
        Avert current obstacle and set state to SAFE
        '''
        self.stop()
        while (rospy.wait_for_message("robot_data", RobotData).state) != "SAFE":
            self.move(0, 0.1)
        rospy.loginfo("Obstacle averted")

    def stop(self):
        '''
        Stop the robot from moving further
        '''
        self.move(0, 0)

    @DeprecationWarning
    def move_towards_target(self):
        '''
        Continuously move towards the target
        '''
        # TODO: Add argument for target in A*
        rospy.loginfo("Aligning robot towards target")
        rospy.logwarn("This could take upto 2-3 minutes in worst case")
        turn_msg = Twist()
        # Align the robot towards the target
        curr_orientation = self.get_current_orientation()
        # temp is the orientation of robot in radians
        temp = ((tf.transformations.euler_from_quaternion((
            curr_orientation.x,
            curr_orientation.y,
            curr_orientation.z,
            curr_orientation.w
        )))[2])
        rospy.loginfo("Current orientation tan: {}".format(
                str(math.tan(temp))))
        # check if atan(m-line_slope) is ~= temp in radians
        while not self.__approx_equals(math.atan(self.mline_slope), temp):
            rospy.loginfo("m_angle: {}, rotation: {}".format(
                math.degrees(math.atan(self.mline_slope)), math.degrees(temp)))
            turn_msg.angular.z = 0.05
            self.vel_pub.publish(turn_msg)
            curr_orientation = self.get_current_orientation()
            temp = ((tf.transformations.euler_from_quaternion((
                curr_orientation.x,
                curr_orientation.y,
                curr_orientation.z,
                curr_orientation.w
                    )))[2])

    def move_towards_target(self, target):
        '''
        Continuously move robot towards target (x, y)
        '''
        # Align towards the target
        self.align_towards_target(target)
        rospy.loginfo("Alignment to local target done")
        current_cell = self.get_current_cell()
        target_cell = inGridCell(target[0], target[1])
        vfh_cost = (rospy.wait_for_message("vfh_cost", Float64)).data
        while not ((current_cell.row == target_cell[1]) and (current_cell.column == target_cell[0])):
            self.move(0.2, 0)
            current_cell = self.get_current_cell()
            vfh_cost = (rospy.wait_for_message("vfh_cost", Float64)).data
            if vfh_cost > 0.8:
                rospy.logwarn("Obstacle found!")
                break

            str_cur_row = str(current_cell.row)
            str_cur_col = str(current_cell.column)
            str_tar_row = target_cell[1]
            str_tar_col = target_cell[0]
            current_cell = self.get_current_cell()
            rospy.loginfo("{} Cur: {} {}; Tar: {} {}".format(str(vfh_cost), str_cur_row, str_cur_col, str_tar_row, str_tar_col))
            current_cell = self.get_current_cell()
            if current_cell.row == self.finalGoal.row and current_cell.column == self.finalGoal.col:
                rospy.loginfo("Somehow reached within 0.5m of final goal, stopping movement!")
                self.stop()
                self.state = states.TARGET_REACHED
                return
            (rospy.Rate(10)).sleep()

        if vfh_cost > 0.7:
            self.avert_obstacle()
        else:
            rospy.loginfo("Reached local target!")

    def __init__(self, finalGoal, name="WhizKID"):
        self.name = name
        rospy.loginfo("Initializing robot {}".format(self.name))
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.current_cell = self.get_current_cell()
        self.current_position = self.get_current_position()
        self.curr_orientation = self.get_current_orientation()
        self.finalGoal = finalGoal
        cur_radians = ((tf.transformations.euler_from_quaternion((
            self.curr_orientation.x,
            self.curr_orientation.y,
            self.curr_orientation.z,
            self.curr_orientation.w
        )))[2])

        init_info = "Initialized robot at ({},{}) in cell ({},{}) with direction {}".format(
            self.current_position.x, self.current_position.y,
            self.current_cell.row, self.current_cell.column, cur_radians)
        rospy.loginfo(init_info)
        rospy.loginfo("Final goal is {} {}".format(str(self.finalGoal.row), str(self.finalGoal.col)))

        self.state = states.GLOBAL_PLANNING
