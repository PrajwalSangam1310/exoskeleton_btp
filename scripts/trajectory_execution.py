#!/usr/bin/env python3

from ctypes.wintypes import HANDLE
from multiprocessing.connection import Listener
from tkinter import X
import rospy
import tf
from trajectory_generator import x_trajectory, z_trajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Transform, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from control_msgs.msg import JointTrajectoryAction,FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction, JointTolerance
from colorama import Fore
from std_msgs.msg import Header
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
from nav_msgs.msg import Path
import copy
from colorama import Fore
import math
from inverse_kinematics import BipedInverseKinematics
from geometry_msgs.msg import Pose
from state_machine import WalkingStateMachine
from exoskeleton_btp.msg import exoState
from control_msgs.msg import FollowJointTrajectoryActionResult

class Walking:
    def __init__(self):
        self.step_length = 0.8
        self.step_forward_length = 0.4
        self.step_height = 0.2
        self.time_step = rospy.Duration(0.05)
        self.robot = moveit_commander.RobotCommander()
        self.knee_stance_flexion = -5

        # self.joint_names =     ['joint_left_abd'
        #                         ,'joint_left_hip_flexion'
        #                         ,'joint_left_knee_flexion'
        #                         ,'joint_left_ankle_flexion'
        #                         ,'joint_right_abd'
        #                         ,'joint_right_hip_flexion'
        #                         ,'right_knee_joint'
        #                         ,'right_ankle_flexion']
        self.joint_names = robot.get_current_state().joint_state.name

        self.walking_state_machine = WalkingStateMachine()
        self.current_state_publisher = rospy.Publisher("exo/curernt_state", exoState, queue_size=10)
        self.current_state = exoState()
        self.next_state_subscriber = rospy.Subscriber("exo/next_state", exoState,self.next_state_cb)

        self.controller_result_topic = "/exo_position_controller/follow_joint_trajectory/result"
        self.controller_result_subscriber = rospy.Subscriber(self.controller_result_topic, FollowJointTrajectoryActionResult, self.controller_result_cb)
        self.execution_status = 0

        self.transition_functions = {
                                        0 : self.waiting,
                                        1 : self.get_right_full_step_joint_trajectory,
                                        2 : self.get_left_full_step_joint_trajectory,
                                        3 : self.get_right_half_step_joint_trajectory,
                                        4 : self.get_left_half_step_joint_trajectory,
                                        5 : self.get_left_front_half_step_joint_trajectory,
                                        6 : self.get_right_front_half_step_joint_trajectory,
        }
        
    def next_state_cb(self,data):
        print("Next state command is", data.state)
        self.walking_state_machine.next_state = data.state

    def controller_result_cb(self,data):
        self.execution_status = 0
        print(Fore.GREEN+"Got controller result"+Fore.RESET)

    def do_transition(self):
        transition_code = self.walking_state_machine.get_transition()
        # print(self.walking_state_machine.state, self.walking_state_machine.next_state)
        if transition_code == 0:
            print(Fore.BLUE + "Waiting..." + Fore.RESET)
            return
        # print("transition code is", transition_code)

        transition_joint_trajectory = self.transition_functions[transition_code]()
        self.walking_state_machine.state = self.walking_state_machine.next_state
        self.current_state.state = self.walking_state_machine.state 
        self.current_state_publisher.publish(self.current_state)
        
        self.execution_status = 1
        self.transition_client(transition_joint_trajectory)

    def do_walk(self):
        if self.execution_status == 1:
            print(Fore.RED+"Busy"+Fore.RESET)
        else:
            self.do_transition()

    def show_traj(self):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        # print(display_trajectory.trajectory_start.joint_state.name)
        display_trajectory.trajectory = []

        robot_traj = moveit_msgs.msg.RobotTrajectory()
        robot_traj.joint_trajectory = JointTrajectory()
        robot_traj.joint_trajectory.joint_names = self.joint_names
        robot_traj.joint_trajectory.points = []

        step_angles = self.get_step_forward_joint_trajectory()
        right_stance_angles = self.get_right_stance_joint_trajectory()
        left_stance_angles = self.get_left_stance_joint_trajectory()
        stop_angles = self.get_right_half_step_joint_trajectory()

        robot_traj.joint_trajectory.points.extend(step_angles.points)
        robot_traj.joint_trajectory.points.extend(left_stance_angles.points)
        robot_traj.joint_trajectory.points.extend(right_stance_angles.points)
        robot_traj.joint_trajectory.points.extend(stop_angles.points)


        display_trajectory.trajectory.append(robot_traj)
        return display_trajectory

    def waiting(self):
        r = rospy.Rate(1)
        # while self.walking_state_machine.state == self.walking_state_machine.next_state:
        print(Fore.BLUE + "Waiting....." + Fore.RESET)

    def get_step_forward_joint_trajectory(self):
        # joint names
        # - joint_left_abd
        # - joint_left_hip_flexion
        # - joint_left_knee_flexion
        # - joint_left_ankle_flexion
        # - joint_right_abd
        # - joint_right_hip_flexion
        # - right_knee_joint
        # - right_ankle_flexion
        
        x = x_trajectory()
        x.initiate_trajectories()
        x.step_length = self.step_forward_length
        x.update_trajectory()

        z = z_trajectory()
        z.initiate_trajectories()
        z.step_height = self.step_height
        z.update_trajectory()

        ik_solver = BipedInverseKinematics()

        hip_pose = Pose()
        hip_pose.position.y = 0.9
        hip_pose.position.x = x.f0[0,0]/2

        left_stance_joint_angles = [0,0,0,0,0,0,0,0]
        leg_length = ik_solver.l2 + ik_solver.l3

        step_forward_joint_trajectory = JointTrajectory()
        step_forward_joint_trajectory.points = []

        for i in range(x.f0.size):
            # swing leg calculation
            angles_left = ik_solver.get_angles(x.f0[0,i]-hip_pose.position.x,hip_pose.position.y - z.f0[0,i])
            left_stance_joint_angles[1] = angles_left[0,0]
            left_stance_joint_angles[2] = angles_left[0,1]

            # stance leg calculation
            hip_pose.position.x = x.f0[0,i]/2
            left_footx = -(x.f0[0,i]-hip_pose.position.x) # calculate the angle of stance leg
            right_hip_flexion =  math.atan2(left_footx, leg_length)
            left_stance_joint_angles[5] = right_hip_flexion
            left_stance_joint_angles[6] = math.radians(self.knee_stance_flexion)

            
            # Adding to the trajectory
            temp = JointTrajectoryPoint()
            temp.positions = copy.deepcopy(left_stance_joint_angles)
            # temp.time_from_start = i*self.time_step
            step_forward_joint_trajectory.points.append(temp)

        return step_forward_joint_trajectory

    def get_left_full_step_joint_trajectory(self):
        # joint names
        # - joint_left_abd
        # - joint_left_hip_flexion
        # - joint_left_knee_flexion
        # - joint_left_ankle_flexion
        # - joint_right_abd
        # - joint_right_hip_flexion
        # - right_knee_joint
        # - right_ankle_flexion
        hip_pose = Pose()
        hip_pose.position.y = 0.9
        
        x = x_trajectory()
        x.initiate_trajectories()
        x.step_length = self.step_length
        x.update_trajectory()
        x.f0 = x.f0 - 0.4

        z = z_trajectory()
        z.initiate_trajectories()
        z.step_height = self.step_height
        z.update_trajectory()

        ik_solver = BipedInverseKinematics()

        hip_pose.position.x = x.f0[0,0]/2

        right_stance_joint_angles = [0,0,0,0,0,0,0,0]
        leg_length = ik_solver.l2 + ik_solver.l3

        right_stance_joint_trajectory = JointTrajectory()
        right_stance_joint_trajectory.points = []

        for i in range(x.f0.size):
            # swing leg calculation
            angles_left = ik_solver.get_angles(x.f0[0,i]-hip_pose.position.x,hip_pose.position.y - z.f0[0,i])
            right_stance_joint_angles[1] = angles_left[0,0]
            right_stance_joint_angles[2] = angles_left[0,1]
            
            # stance leg calculation
            hip_pose.position.x = x.f0[0,i]/2
            right_footx = -(x.f0[0,i]-hip_pose.position.x) # calculate the angle of stance leg
            right_hip_flexion =  math.atan2(right_footx, leg_length)
            right_stance_joint_angles[5] = right_hip_flexion
            right_stance_joint_angles[6] = math.radians(self.knee_stance_flexion)

            
            # Adding to the trajectory
            temp = JointTrajectoryPoint()
            temp.positions = copy.deepcopy(right_stance_joint_angles)
            # temp.time_from_start = i*self.time_step
            right_stance_joint_trajectory.points.append(temp)

        return right_stance_joint_trajectory

    def get_right_full_step_joint_trajectory(self):
        # joint names
        # - joint_left_abd
        # - joint_left_hip_flexion
        # - joint_left_knee_flexion
        # - joint_left_ankle_flexion
        # - joint_right_abd
        # - joint_right_hip_flexion
        # - right_knee_joint
        # - right_ankle_flexion
        hip_pose = Pose()
        hip_pose.position.y = 0.9

        x = x_trajectory()
        x.initiate_trajectories()
        x.step_length = self.step_length
        x.update_trajectory()
        x.f0 = x.f0 - 0.4

        z = z_trajectory()
        z.initiate_trajectories()
        z.step_height = self.step_height
        z.update_trajectory()

        ik_solver = BipedInverseKinematics()

        hip_pose.position.x = x.f0[0,0]/2

        left_stance_joint_angles = [0,0,0,0,0,0,0,0]
        leg_length = ik_solver.l2 + ik_solver.l3

        left_stance_joint_trajectory = JointTrajectory()
        left_stance_joint_trajectory.points = []

        for i in range(x.f0.size):
            # swing leg calculation
            angles_left = ik_solver.get_angles(x.f0[0,i]-hip_pose.position.x,hip_pose.position.y - z.f0[0,i])
            left_stance_joint_angles[5] = -angles_left[0,0]
            left_stance_joint_angles[6] = -angles_left[0,1]
            
            # tance leg calculation
            hip_pose.position.x = x.f0[0,i]/2
            left_footx = -(x.f0[0,i]-hip_pose.position.x) # calculate the angle of stance leg
            left_hip_flexion =  math.atan2(left_footx, leg_length)
            left_stance_joint_angles[1] = -left_hip_flexion
            left_stance_joint_angles[2] = -math.radians(self.knee_stance_flexion)
            
            # adding to the trajectory
            temp = JointTrajectoryPoint()
            temp.positions = copy.deepcopy(left_stance_joint_angles)
            # temp.time_from_start = i*self.time_step
            left_stance_joint_trajectory.points.append(temp)

        return left_stance_joint_trajectory
    
    def get_left_half_step_joint_trajectory(self):
        hip_pose = Pose()
        hip_pose.position.y = 0.9
        
        x = x_trajectory()
        x.initiate_trajectories()
        x.step_length = self.step_length/2
        x.update_trajectory()
        x.f0 = x.f0 - 0.4

        z = z_trajectory()
        z.initiate_trajectories()
        z.step_height = self.step_height
        z.update_trajectory()

        ik_solver = BipedInverseKinematics()

        hip_pose.position.x = x.f0[0,0]/2

        right_stance_joint_angles = [0,0,0,0,0,0,0,0]
        leg_length = ik_solver.l2 + ik_solver.l3

        right_stance_joint_trajectory = JointTrajectory()
        right_stance_joint_trajectory.points = []

        for i in range(x.f0.size):
            # swing leg calculation
            angles_left = ik_solver.get_angles(x.f0[0,i]-hip_pose.position.x,hip_pose.position.y - z.f0[0,i])
            right_stance_joint_angles[1] = angles_left[0,0]
            right_stance_joint_angles[2] = angles_left[0,1]
            
            # stance leg calculation
            hip_pose.position.x = x.f0[0,i]/2
            right_footx = -(x.f0[0,i]-hip_pose.position.x) # calculate the angle of stance leg
            right_hip_flexion =  math.atan2(right_footx, leg_length)
            right_stance_joint_angles[5] = right_hip_flexion
            right_stance_joint_angles[6] = math.radians(self.knee_stance_flexion)

            
            # Adding to the trajectory
            temp = JointTrajectoryPoint()
            temp.positions = copy.deepcopy(right_stance_joint_angles)
            # temp.time_from_start = i*self.time_step
            right_stance_joint_trajectory.points.append(temp)

        return right_stance_joint_trajectory

    def get_right_half_step_joint_trajectory(self):
        # joint names
        # - joint_left_abd
        # - joint_left_hip_flexion
        # - joint_left_knee_flexion
        # - joint_left_ankle_flexion
        # - joint_right_abd
        # - joint_right_hip_flexion
        # - right_knee_joint
        # - right_ankle_flexion
        hip_pose = Pose()
        hip_pose.position.y = 0.9

        x = x_trajectory()
        x.initiate_trajectories()
        x.step_length = self.step_length/2
        x.update_trajectory()
        x.f0 = x.f0 - 0.4

        z = z_trajectory()
        z.initiate_trajectories()
        z.step_height = self.step_height
        z.update_trajectory()

        ik_solver = BipedInverseKinematics()

        hip_pose.position.x = x.f0[0,0]/2

        left_stance_joint_angles = [0,0,0,0,0,0,0,0]
        leg_length = ik_solver.l2 + ik_solver.l3

        left_stance_joint_trajectory = JointTrajectory()
        left_stance_joint_trajectory.points = []

        for i in range(x.f0.size):
            # swing leg calculation
            angles_left = ik_solver.get_angles(x.f0[0,i]-hip_pose.position.x,hip_pose.position.y - z.f0[0,i])
            left_stance_joint_angles[5] = -angles_left[0,0]
            left_stance_joint_angles[6] = -angles_left[0,1]
            
            # tance leg calculation
            hip_pose.position.x = x.f0[0,i]/2
            left_footx = -(x.f0[0,i]-hip_pose.position.x) # calculate the angle of stance leg
            left_hip_flexion =  math.atan2(left_footx, leg_length)
            left_stance_joint_angles[1] = -left_hip_flexion
            left_stance_joint_angles[2] = -math.radians(self.knee_stance_flexion)
            
            # adding to the trajectory
            temp = JointTrajectoryPoint()
            temp.positions = copy.deepcopy(left_stance_joint_angles)
            # temp.time_from_start = i*self.time_step
            left_stance_joint_trajectory.points.append(temp)

        return left_stance_joint_trajectory

    def get_right_front_half_step_joint_trajectory(self):
        # joint names
        # - joint_left_abd
        # - joint_left_hip_flexion
        # - joint_left_knee_flexion
        # - joint_left_ankle_flexion
        # - joint_right_abd
        # - joint_right_hip_flexion
        # - right_knee_joint
        # - right_ankle_flexion
        
        x = x_trajectory()
        x.initiate_trajectories()
        x.step_length = self.step_forward_length
        x.update_trajectory()

        z = z_trajectory()
        z.initiate_trajectories()
        z.step_height = self.step_height
        z.update_trajectory()

        ik_solver = BipedInverseKinematics()

        hip_pose = Pose()
        hip_pose.position.y = 0.9
        hip_pose.position.x = x.f0[0,0]/2

        left_stance_joint_angles = [0,0,0,0,0,0,0,0]
        leg_length = ik_solver.l2 + ik_solver.l3

        step_forward_joint_trajectory = JointTrajectory()
        step_forward_joint_trajectory.points = []

        for i in range(x.f0.size):
            # swing leg calculation
            angles_left = ik_solver.get_angles(x.f0[0,i]-hip_pose.position.x,hip_pose.position.y - z.f0[0,i])
            left_stance_joint_angles[1] = angles_left[0,0]
            left_stance_joint_angles[2] = angles_left[0,1]

            # stance leg calculation
            hip_pose.position.x = x.f0[0,i]/2
            left_footx = -(x.f0[0,i]-hip_pose.position.x) # calculate the angle of stance leg
            right_hip_flexion =  math.atan2(left_footx, leg_length)
            left_stance_joint_angles[5] = right_hip_flexion
            left_stance_joint_angles[6] = math.radians(self.knee_stance_flexion)

            
            # Adding to the trajectory
            temp = JointTrajectoryPoint()
            temp.positions = copy.deepcopy(left_stance_joint_angles)
            # temp.time_from_start = i*self.time_step
            step_forward_joint_trajectory.points.append(temp)

        return step_forward_joint_trajectory

    def get_left_front_half_step_joint_trajectory(self):
        # joint names
        # - joint_left_abd
        # - joint_left_hip_flexion
        # - joint_left_knee_flexion
        # - joint_left_ankle_flexion
        # - joint_right_abd
        # - joint_right_hip_flexion
        # - right_knee_joint
        # - right_ankle_flexion
        
        x = x_trajectory()
        x.initiate_trajectories()
        x.step_length = self.step_forward_length
        x.update_trajectory()

        z = z_trajectory()
        z.initiate_trajectories()
        z.step_height = self.step_height
        z.update_trajectory()

        ik_solver = BipedInverseKinematics()

        hip_pose = Pose()
        hip_pose.position.y = 0.9
        hip_pose.position.x = x.f0[0,0]/2

        left_stance_joint_angles = [0,0,0,0,0,0,0,0]
        leg_length = ik_solver.l2 + ik_solver.l3

        step_forward_joint_trajectory = JointTrajectory()
        step_forward_joint_trajectory.points = []

        for i in range(x.f0.size):
            # swing leg calculation
            angles_left = ik_solver.get_angles(x.f0[0,i]-hip_pose.position.x,hip_pose.position.y - z.f0[0,i])
            left_stance_joint_angles[5] = -angles_left[0,0]
            left_stance_joint_angles[6] = -angles_left[0,1]

            # stance leg calculation
            hip_pose.position.x = x.f0[0,i]/2
            left_footx = -(x.f0[0,i]-hip_pose.position.x) # calculate the angle of stance leg
            right_hip_flexion =  math.atan2(left_footx, leg_length)
            left_stance_joint_angles[1] = -right_hip_flexion
            left_stance_joint_angles[2] = -math.radians(self.knee_stance_flexion)

            
            # Adding to the trajectory
            temp = JointTrajectoryPoint()
            temp.positions = copy.deepcopy(left_stance_joint_angles)
            # temp.time_from_start = i*self.time_step
            step_forward_joint_trajectory.points.append(temp)

        return step_forward_joint_trajectory

    def transition_client(self, transition_joint_trajectroy):
        # walking_trajectory = JointTrajectory()
        transition_joint_trajectroy.header = Header()
        transition_joint_trajectroy.header.stamp = rospy.Time.now()
        transition_joint_trajectroy.joint_names = self.joint_names
        for i in range(len(transition_joint_trajectroy.points)):
            transition_joint_trajectroy.points[i].time_from_start = i*self.time_step

        # Creates the SimpleActionClient, passing the type of the action
        print("before client")
        client = actionlib.SimpleActionClient('/exo_position_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("after client")

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
        
        # Creates a goal to send to the action server.
        goal = FollowJointTrajectoryActionGoal()
        print("goal is initialized")
        goal.goal.trajectory = transition_joint_trajectroy

        #set the trajectory points
        goal.goal.path_tolerance = [JointTolerance(self.joint_names[i],0,0,0) for i in range(len(self.joint_names))]
        goal.goal.goal_tolerance = [JointTolerance(self.joint_names[i],0,0,0) for i in range(len(self.joint_names))]
        goal.goal.goal_time_tolerance = rospy.Duration(0.1)

        # Sends the goal to the action server.
        client.send_goal(goal.goal)
        print(Fore.BLUE + "Transition has been assigned" + Fore.RESET)

    def get_walking_trajectory(self):
        walking_trajectory = JointTrajectory()
        walking_trajectory.header = Header()
        walking_trajectory.header.stamp = rospy.Time.now()

        walking_trajectory.joint_names = self.joint_names
        walking_trajectory.points = []

        step_angles = self.get_step_forward_joint_trajectory()
        right_stance_angles = self.get_right_full_step_joint_trajectory()
        left_stance_angles = self.get_left_full_step_joint_trajectory()

        walking_trajectory.points.extend(step_angles.points)
        walking_trajectory.points.extend(left_stance_angles.points)
        walking_trajectory.points.extend(right_stance_angles.points)
        # print(walking_trajectory.points)
        for i in range(len(walking_trajectory.points)):
            walking_trajectory.points[i].time_from_start = i*self.time_step

        return walking_trajectory

    def full_walking_client(self):
        # Creates the SimpleActionClient, passing the type of the action
        print("before client")
        client = actionlib.SimpleActionClient('/exo_position_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("after client")

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
        
        # Creates a goal to send to the action server.
        goal = FollowJointTrajectoryActionGoal()
        print("goal is initialized")
        goal.goal.trajectory = self.get_walking_trajectory()
        #set the trajectory points
        goal.goal.path_tolerance = [JointTolerance(self.joint_names[i],0,0,0) for i in range(len(self.joint_names))]
        goal.goal.goal_tolerance = [JointTolerance(self.joint_names[i],0,0,0) for i in range(len(self.joint_names))]
        goal.goal.goal_time_tolerance = rospy.Duration(0.1)

        # Sends the goal to the action server.
        client.send_goal(goal.goal)


# algo 
# first plan for the the right foot for x/2.
# take the same x and use sin-1(x/total_length) to get the hip flexion of the robot
# then use the plan group to finish the thing

# walking flow, once that trajectory is completed activate some kind of flag or same phase
# is execting 
#   if not stop
#   execute the next goal

# create the state machine
# goal walking sitting interrupt, trajectory change etc

# inerface with the pyqt gui

if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('execution_node', anonymous=True)
    robot = moveit_commander.RobotCommander()

    left_leg_group = moveit_commander.MoveGroupCommander("left_leg")
    right_leg_group = moveit_commander.MoveGroupCommander("right_leg")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=10)
 
    left_topic_name = "exoskeleton/LeftAnkleTraj"
    right_topic_name = "exoskeleton/RightAnkleTraj"
    #creating inversekinematic object
    
    walk = Walking()
    # display_trajectory = walk.show_traj()
    # walk.walking_client()
    print(Fore.BLUE+"Calculation completed"+ Fore.RESET)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        # display_trajectory_publisher.publish(display_trajectory)
        walk.do_walk()
        # print(Fore.GREEN+"In while Loop"+ Fore.RESET)
        r.sleep()


    

