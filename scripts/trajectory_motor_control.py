#!/usr/bin/env python3

from asyncio import QueueEmpty
import rospy
from trajectory_generator import x_trajectory, z_trajectory
from colorama import Fore
import sys
from nav_msgs.msg import Path
import copy
import math
from inverse_kinematics import BipedInverseKinematics
from state_machine import WalkingStateMachine
from exoskeleton_btp.msg import exoState,exoAllJointTrajectory,walkingParameters
import numpy as np
"""
@breif: 
- subscribe to current and next state.
- if update button is pressed.
    - check if next change is changed, update the trajectory.
- Publishing data.
    - while publishing just publish on the command msg goal velocity and goal position
- Publishing timings.
    - either publish through rospy time or while loop or timer function.
"""


class HighLevelController:
    def __init__(self):
        self.walking_state_machine = WalkingStateMachine()

        self.current_state_publisher = rospy.Publisher("exo/curernt_state", exoState, queue_size=10)
        self.current_state = exoState()

        self.next_state = exoState()
        self.next_state_subscriber = rospy.Subscriber("exo/next_state", exoState,self.next_state_cb)

        """trajectory variables"""
        self.walking_params = walkingParameters()
        self.walking_param_suibscriber = rospy.Subscriber("exo_walking_parameters", walkingParameters, self.walking_cb)
        self.inverseKinematicSolver = BipedInverseKinematics()
        self.stateMachine = WalkingStateMachine()

        self.z = z_trajectory()
        self.x = x_trajectory()

        """ hardwared topics"""
        self.right_thigh_index = 1
        self.right_knee_index = 2

        self.feedback_topic = "exo/feedback_data"
        self.command_topic = "exo/trajectory"


        self.trajectory_msg = exoAllJointTrajectory()

        self.trajectory_publisher = rospy.Publisher(self.command_topic, exoAllJointTrajectory, queue_size= 10)
        self.left_stance = False
        self.right_stance = False


    def walking_cb(self,data):
        self.walking_params.stepHeight = data.stepHeight
        self.walking_params.stepLength = data.stepLength
        self.walking_params.stepTime = data.stepTime


    def next_state_cb(self,data):
        # if(not self.next_state.state == data.state):
        #     self.next_state.state = data.state
        self.update_test_trajectories()
    
    def update_trajectories(self):
        print("updating new trajectories")
        self.no_of_points = 200
        
        self.x.no_of_trajectory_points = self.no_of_points
        self.x.step_length = self.walking_params.stepLength
        self.x.step_time = self.walking_params.stepTime
        self.x.stop_time = self.x.step_time


        self.z.no_of_trajectory_points = self.no_of_points
        self.z.step_time = self.walking_params.stepTime
        self.z.step_height = self.walking_params.stepHeight
        self.z.stop_time = self.z.step_time

        r = rospy.Rate(self.no_of_points/self.x.step_time)
        
        self.x.initiate_trajectories()
        self.x.update_trajectory()
        self.x.f0 = self.x.f0 - 0.25
        self.x.show_trajectories()


        self.z.initiate_trajectories()
        self.z.update_trajectory()
        self.z.show_trajectories()

        for i in range(self.no_of_points):
            t = i*self.x.step_time/self.no_of_points
            angles = [math.degrees(theta) for theta in self.inverseKinematicSolver.get_angles(self.x.f0[0,i], self.z.f0[0,i])[0]]
            print(angles)
            self.trajectory_msg.goal_position[self.right_thigh_index] = angles[0]
            self.trajectory_msg.goal_position[self.right_knee_index] = angles[1]
            self.publish_commands()
            r.sleep()
        
    def update_test_trajectories(self):
        self.no_of_points = 200
        print(self.right_stance, self.left_stance)
        r = rospy.Rate(self.no_of_points/self.x.step_time)
        if self.right_stance:
            theta1 = np.linspace(-30, 30, self.no_of_points)
        elif self.left_stance:
            theta1 = np.linspace(30, -30, self.no_of_points)
        else:
            theta1 = np.linspace(0, 30, self.no_of_points)

        if self.right_stance or self.right_stance:
            theta2 = np.append(np.linspace(0,-55, int(self.no_of_points/2)), np.linspace(-55,0, int(self.no_of_points/2)))
        else:
            theta2 = np.append(np.linspace(0,-30, int(self.no_of_points/2)), np.linspace(-30,0, int(self.no_of_points/2)))
            self.right_stance = True    

        if self.left_stance:
            self.left_stance = False
            self.right_stance = True
        
        elif self.right_stance:
            self.right_stance = False
            self.left_stance = True
        
        for i in range(self.no_of_points):
            self.trajectory_msg.goal_position[self.right_thigh_index] = theta1[i]
            self.trajectory_msg.goal_position[self.right_knee_index] = theta2[i]
            print(theta1[i],theta2[i])
            self.publish_commands()
            r.sleep()  
        
    def publish_commands(self):
        self.trajectory_publisher.publish(self.trajectory_msg)
        # self.right_thigh_command_publisher.publish(self.right_thigh_command_msg)
        
if __name__ == "__main__":
    rospy.init_node("trajectory_motor_control", anonymous=True)
    hc = HighLevelController()
    rate = rospy.Rate(0.13) 
    ## education minister changes####
    while not rospy.is_shutdown():
        hc.update_test_trajectories()
        rate.sleep()
    ######################
    # rospy.spin()
