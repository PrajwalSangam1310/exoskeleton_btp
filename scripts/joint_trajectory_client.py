#!/usr/bin/env python3

from genpy.rostime import Duration
import rospy
# from __future__ import print_function
import sys
import actionlib
from control_msgs.msg import JointTrajectoryAction,FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction, JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from std_msgs.msg import Header, Time
import rospkg
import csv
import math

class RightLeg:
    def __init__(self):
        self.jointNames = [ 
                            'joint_right_abd'
                            ,'joint_right_ir'
                            ,'joint_right_hip_flexion'
                            ,'right_knee_joint'
                            ,'right_ankle_flexion'
                            ]

class LeftLeg:
    def __init__(self):
        self.jointNames = [ 
                            'joint_left_abd'
                            ,'joint_left_ir'
                            ,'joint_left_hip_flexion'
                            ,'joint_left_knee_flexion'
                            ,'joint_left_ankle_flexion'
                            ]
        self.legJointTrajectory = JointTrajectory()
        self.trajectoryPoints = None
        self.frameId = 'base_link'
        self.noOfJoints = len(self.jointNames)
        self.noOfTrajectoryPoints = 0
        self.fileName = 'subject01_walk1_ik.csv'


    def loadTrajectoryPoints(self):
        rospack = rospkg.RosPack()  
        leftkeysName = ['hip_adduction_l', 'hip_rotation_l', 'hip_flexion_l','knee_angle_l', 'ankle_angle_l']
        rightkeysName = ['hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r','knee_angle_r', 'ankle_angle_r']
        self.trajectoryPoints = None
        path = rospack.get_path('exoskeleton_btp')
        path = path + '/trajectories/' + self.fileName
        with open(path, mode='r', encoding='utf-8-sig') as csv_file:
                    csv_reader = csv.DictReader(csv_file)
                    N = 0
                    for row in csv_reader:
                        N += 1
                    print('the size is',N)      

        self.trajectoryPoints = np.zeros((N, 5))
        with open(path, mode='r', encoding='utf-8-sig') as csv_file:
                    csv_reader = csv.DictReader(csv_file)
                    i =0
                    for row in csv_reader:
                        j = 0
                        for key in leftkeysName:
                            temp = row[key]
                            self.trajectoryPoints[i][j] = math.radians(float(temp))
                            # print(row[key], end=" ")
                            j += 1
                        # print(" ")
                        i += 1

        self.noOftrajectoryPoints = self.trajectoryPoints.shape[0]

    def updteTrajectory2(self):
        print("in update trajectory 2")
        for i in range(len(self.legJointTrajectory.points)):
            # print(self.legJointTrajectory.points[i])
            for j in range(5):
                self.legJointTrajectory.points[i].positions[j] = 0
        #adduction test
        adductionjointAngles = np.linspace(-math.pi/4, math.pi/4, len(self.legJointTrajectory.points))
        for i in range(len(self.legJointTrajectory.points)): 
            # self.legJointTrajectory.points[i].positions[1] = adductionjointAngles[i]   
            self.legJointTrajectory.points[i].positions[0] = 0          
            self.legJointTrajectory.points[i].positions[1] = 0       
            self.legJointTrajectory.points[i].positions[2] = 0       
            self.legJointTrajectory.points[i].positions[3] = math.pi/2   
            self.legJointTrajectory.points[i].positions[4] = math.pi


    def initiateLegJointTrajectory(self):
        print("inside the initiate leg joint trajectory")
        #initiate the jointTrajectory
        self.legJointTrajectory = JointTrajectory()

        # initiate the header
        self.legJointTrajectory.header = Header()
        self.legJointTrajectory.header.seq = 0
        self.legJointTrajectory.header.stamp = rospy.Time.now()
        # print(self.legJointTrajectory.header.stamp)
        self.legJointTrajectory.header.frame_id = self.frameId

        #set the joint names
        self.legJointTrajectory.joint_names = self.jointNames

        #make the jointPoints positions, velocities and accelerationsto 0
        self.legJointTrajectory.points = [JointTrajectoryPoint() for i in range(self.trajectoryPoints.shape[0])]
        
        # initiate each trajectory point to zero
        tempTime = rospy.Duration(0) 
        dt = rospy.Duration(1)
        for i in range(len(self.legJointTrajectory.points)):
            self.legJointTrajectory.points[i].positions = [0.0 for i in range(self.noOfJoints)]
            self.legJointTrajectory.points[i].velocities = [0.0 for i in range(self.noOfJoints)]
            self.legJointTrajectory.points[i].accelerations = [0.0 for i in range(self.noOfJoints)]
            self.legJointTrajectory.points[i].effort = [0.0 for i in range(self.noOfJoints)]
            
            # set the time from start
            self.legJointTrajectory.points[i].time_from_start = tempTime
            tempTime = tempTime + dt 
            # print(tempTime)

    def updateLegJointTrajectory(self):
        deltaT = 0.1
        for i in range(len(self.legJointTrajectory.points)):
            for j in range(5):
                self.legJointTrajectory.points[i].positions[j] = self.trajectoryPoints[i][j]
                # print(self.legJointTrajectory.points[i].positions[j], end=" ")
            # print("int update function")
        # self.legJointTrajectory.points[i].positions[j] = 2

    def checkTrajectory(self):
        print("in check trajectory")
        for i in range(len(self.legJointTrajectory.points)):
            # print(self.legJointTrajectory.points[i])
            print(self.legJointTrajectory.points[i].positions)
        print(self.legJointTrajectory.joint_names)
    
    def simpleTest(self):
        print(JointTrajectoryPoint())


    def legJointTrajectoryClient(self):
        # Creates the SimpleActionClient, passing the type of the action
        print("before client")
        client = actionlib.SimpleActionClient('/left_leg_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("after client")
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = FollowJointTrajectoryActionGoal()
        print("goal is initialized")
        goal.goal.trajectory = self.legJointTrajectory
        #set the trajectory points
        goal.goal.path_tolerance = [JointTolerance(self.jointNames[i],0,0,0) for i in range(len(self.jointNames))]
        goal.goal.goal_tolerance = [JointTolerance(self.jointNames[i],0,0,0) for i in range(len(self.jointNames))]
        goal.goal.goal_time_tolerance = rospy.Duration(0.1)
        
        # Sends the goal to the action server.
        # print(goal.goal.trajectory)
        client.send_goal(goal.goal)
        print("goal assigning is passed")
        client.wait_for_result()

    def jointTrajectoryControllerPublisher(self):
        # Creates a goal to send to the action server. 
        topicName = '/left_leg_trajectory_controller/command'

        pub = rospy.Publisher(topicName, JointTrajectory,queue_size=10)    
        pub.publish(self.legJointTrajectory)
        rate = rospy.Rate(10)
        i = 0
        while i<5:
            self.legJointTrajectory.header.stamp = rospy.Time.now()
            pub.publish(self.legJointTrajectory)
            print('publishing the joint trahectory')
            i +=1
            print(i)
            rate.sleep()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('joint_trajectory_client_py', anonymous=True)
        # rleg = RightLeg()
        lleg = LeftLeg()
        lleg.loadTrajectoryPoints()
        lleg.initiateLegJointTrajectory()
        # lleg.updateLegJointTrajectory()
        lleg.updteTrajectory2()
        lleg.checkTrajectory()
        result = lleg.legJointTrajectoryClient()
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)