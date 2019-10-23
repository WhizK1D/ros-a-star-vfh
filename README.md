# ros-a-star-vfh
Implement a navigational robot that performs global planning using A-star and local planning using Vector Field Histogram (VFH) to help navigate the robot from start to end for a given map.

## File Summary:
- launch/pa2.launch: Basic launch file launching all nodes
- msg/: Custom message definitions
- a-star-utils.py: Set of utilities required to run basic A-star path finding
- a-star-launcher: Actual launcher integrating robot control and A-star
- robot.py: Utilities for controlling robot
- overwatch.py: Basic monitoring node to provide the robot tailored information
- vfh.py: Basic VFH implementation that provides different costs

