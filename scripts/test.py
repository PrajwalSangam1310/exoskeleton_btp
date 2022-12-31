#!/usr/bin/env python3 

from genpy.rostime import Duration
import rospkg
import rospy
import csv
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction, JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, Time
import numpy as np



print("inside the script")
# node = rospy.init_node('test_node', anonymous=False)


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
        leftkeysName = ['hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l','knee_angle_l', 'ankle_angle_l']
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
                            self.trajectoryPoints[i][j] = temp
                            j += 1
                        i += 1

        self.noOftrajectoryPoints = self.trajectoryPoints.shape[0]


    def initiateLegJointTrajectory(self):
        #initiate the jointTrajectory
        self.legJointTrajectory = JointTrajectory()

        # initiate the header
        self.legJointTrajectory.header = Header()
        self.legJointTrajectory.header.seq = 0
        self.legJointTrajectory.header.stamp = Time()
        self.legJointTrajectory.header.frame_id = self.frameId

        #set the joint names
        self.legJointTrajectory.joint_names = self.jointNames

        #make the jointPoints positions, velocities and accelerationsto 0
        self.legJointTrajectory.points = [JointTrajectoryPoint for i in range(self.trajectoryPoints.shape[0])]
        
        # initiate each trajectory point to zero
        for i in range(len(self.legJointTrajectory.points)):
            self.legJointTrajectory.points[i].positions = [0.0 for i in range(self.noOfJoints)]
            self.legJointTrajectory.points[i].velocities = [0.0 for i in range(self.noOfJoints)]
            self.legJointTrajectory.points[i].accelerations = [0.0 for i in range(self.noOfJoints)]
            # set the time from start
            self.legJointTrajectory.points[i].time_from_start = rospy.Duration(0.1)

    def updateLegJointTrajectory(self):
        for i in range(len(self.legJointTrajectory.points)):
            for j in range(5):
                self.legJointTrajectory.points[i].positions[j] = self.trajectoryPoints[i][j]

# if __name__ == '__main__':
if False:
    leg = LeftLeg()
    print("in the main")
    leg.loadTrajectoryPoints()
    leg.initiateLegJointTrajectory()
    leg.updateLegJointTrajectory()
    temp = FollowJointTrajectoryActionGoal()
    temp.goal.trajectory = leg.legJointTrajectory
    print(temp.goal.trajectory)

if True:
    fileName = 'subject01_walk1_ik.csv'
    rospack = rospkg.RosPack()  
    leftkeysName = ['hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l','knee_angle_l', 'ankle_angle_l']
    rightkeysName = ['hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r','knee_angle_r', 'ankle_angle_r']
    trajectoryPoints = None
    path = rospack.get_path('exoskeleton_btp')
    path = path + '/trajectories/' + fileName

    testtemp  = JointTolerance(['naems'],[0],[0],[0])
    print(testtemp)