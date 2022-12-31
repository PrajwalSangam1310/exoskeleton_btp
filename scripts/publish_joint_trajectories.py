#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float64
from control_msgs.msg import JointTrajectoryActionGoal
from control_msgs.msg import JointTrajectoryActionResult

rospy.init_node('my_node', anonymous=True)
pub = rospy.Publisher('my_topic', Float64MultiArray, queue_size=10)
