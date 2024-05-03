#!/usr/bin/env python3
import pyassimp
import moveit_commander
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import Marker

import sys
import rospy
import moveit_commander
import tf
try:
    scene = pyassimp.load("/home/kyle/catkin_ws/src/xarm_ros/xarm_gazebo/worlds/FullAssemblyVirtual_two.STL")
    # Do something with the scene if needed
    pyassimp.release(scene)
except pyassimp.errors.AssimpError as e:
    print(f"Error loading STL file: {e}")

