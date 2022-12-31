#!/usr/env/bin python3

""" architecture : process handler -> process -> process.run()"""

from sqlalchemy import false
from trajectory_generator import x_trajectory, z_trajectory
from inverse_kinematics import BipedInverseKinematics
from geometry_msgs.msg import Pose
import math
import numpy as np
import rospy

class ProcessHandler:

    """ Procss has the list of process, it handles the one level higher action from the proccess"""
    def __init__(self):
        self.processes = {0: Standing(),
                         1: Walking(),
                         2: Sit(),
                         3: SitToStand()}
        
        self.curr_process_id = 0
        self.req_process_id = 0
        self.trajetctory_joint_command = np.zeros([2,6])

    def update_process(self):
        if self.curr_process_id != self.req_process_id:
            self.change_process()
        else:
            self.do_process()
            # self.processes[self.curr_process_id].is_at_end = False

    def update_req_id(self, id):
        self.req_process_id = id

    def do_process(self):
        self.processes[self.curr_process_id].run()
        # self.trajectory_joint_command = self.processes[self.curr_process_id].get_process_joint_commands()
        
    def change_process(self):
        if self.processes[self.curr_process_id].is_at_end:
            print("curr process has ended")
            self.processes[self.curr_process_id].reinitialize_trajectories()
            self.processes[self.curr_process_id].is_at_end = False
            self.processes[self.curr_process_id].end_process = False
            self.curr_process_id = self.req_process_id
            self.processes[self.curr_process_id].play = True
            self.processes[self.curr_process_id].stop = False
            self.processes[self.curr_process_id].pause = False
            self.processes[self.curr_process_id].end_process = False
        else:
            self.processes[self.curr_process_id].end_process = True
            self.processes[self.curr_process_id].play = False
            self.processes[self.curr_process_id].stop = True
            self.do_process()
    
    def pause_curr_process(self):
        self.processes[self.curr_process_id].pause = True

    def play_curr_process(self):
        self.processes[self.curr_process_id].play = True

    def stop_curr_process(self):
        self.processes[self.curr_process_id].stop = True

    
class Process:
    """ 
        @brief 
        All the exoskeleton features are defined as processes. 
            Standing is the process, walking is another different process, stair climb is another one... etc 
        This forms the base class for all the processes that will be added in the future, for example u can add turn right as another process.
        """
    def __init__(self):
        #give different process id for each process.
        self.process_id = 0 # default 0 equals to stand and do nothing
        self.process_parameters = None #none for standing
        self.play = True
        self.pause = False
        self.stop = False
        self.is_at_end = False
        self.end_process = False

        self.cur_joint_positions = [0.0 for i in range(6)] # 6 joint angles.
        self.cur_joint_velocities = [0.0 for i in range(6)] # 6 joint velocities.  
        self.command_joint_angles = np.zeros([1,6])
        self.curr_states = [0]
        self.ik_solver = BipedInverseKinematics()

    def update_process_parameters(self, array_of_parameters):
        pass

    def update_curr_variables(self, array_of_curr_state):
        # variables are the joint positions and the velocities.
        for i in range(array_of_curr_state):
            self.cur_joint_positions[i] = array_of_curr_state[i]

    def update_process_joint_commands(self):
        pass

    def get_process_joint_commands(self):
        pass

    def run(self):
        # proces.run() will do all the tasks.
        pass

    def update_joint_velocities(self):
        # update the velocities using difference in the joint angles
        pass

    def reinitialize_trajectories(self):
        # reinitialize the trajectories when the process ends successfully
        pass

class Standing(Process):
    def __init__(self):
        super().__init__()
        self.zero_position = [0.0 for i in range(6)]
        self.zero_velocity = [0.0 for i in range(6)]
        self.is_at_end = True

    def run(self):
        print("Standing still")
        self.is_at_end = True
    
    def get_process_joint_commands(self):
        return np.zeros([2,6])

class Walking(Process):
    def __init__(self):
        super().__init__()
        self.process_parameters = [0.5, 4] 
        self.curr_state_names = ["Stand", "left_stance", "right_stance", "walk"]
        # states are 1,2,3,4

        self.all_transitions = [0,1,2,3,4]
        # 0 for dont do anyting
        # 1 for full_step_right
        # 2 for full_step_left
        # 3 for half_step_right for stopping 
        # 4 for half_step_left for stopping
        # 5 for half_step_right for starting
        # 6 for half_step_left for starting

        self.transition_matrix = [
                                    [0,5,6],
                                    [4,0,2],
                                    [3,1,0]
                                ]
        self.curr_state = 0 # start with stopping
        self.next_state = 0

        self.stop = False
        self.play = True
        self.pause = False

        ## trajectory components
        self.x_trajectory = x_trajectory()
        self.x_trajectory.initiate_trajectories()
        self.z_trajectory = z_trajectory()
        self.z_trajectory.initiate_trajectories()
        self.hip_positionY = 0.912
        self.wlaking_speed_constant = 2
        self.point_publish_rate = 4

    def run(self):
        if self.play:
            self.next_state = self.get_next_state(self.curr_state)
            self.update_trajectory()
            self.curr_state = self.next_state
            
        elif self.stop:
            print("Received Stop command... changing to standing position")
            self.play = False
            self.next_state = 0
            self.update_trajectory()
            self.curr_state = 0

            if self.end_process:
                self.is_at_end = True
                # self.command_joint_angles = np.zeros([1,6])
                # self.update_trajectory()
                print("The Walking process process has ended")
    
    def get_next_state(self, state):
        if self.stop:
            return 0
        if state == 1:
            return 2
        if state == 2:
            return 1
        if state == 0 and self.play:
            return 1
        if state == 0 and self.stop:
            return 0

    def update_trajectory(self):
        if self.curr_state == 0 and self.end_process:
            self.command_joint_angles = np.zeros([1,6])
        
        # updating the trajectory parameters
        step_time =  self.wlaking_speed_constant / self.process_parameters[1] 
        self.x_trajectory.step_time = step_time
        self.x_trajectory.stop_time = step_time

        self.z_trajectory.step_time = step_time
        self.z_trajectory.stop_time = step_time
        self.z_trajectory.step_height = self.process_parameters[2]

        self.x_trajectory.no_of_trajectory_points = int(step_time * self.point_publish_rate)
        self.z_trajectory.no_of_trajectory_points = self.x_trajectory.no_of_trajectory_points

        print("no of points are", self.x_trajectory.no_of_trajectory_points)

        transition = self.transition_matrix[self.curr_state][self.next_state]
        # if transition == 3 or transition == 4 or transition == 5 or transition == 6:
        if transition > 2:
            self.x_trajectory.step_length = self.process_parameters[0]/2
            if transition == 3:
                print("transition: half left step stopping")
            elif transition == 4:
                print("transition: half right step stopping")
        else:
            self.x_trajectory.step_length = self.process_parameters[0]
            if transition == 1:
                print("transition:",transition,"full right step")
            else:
                print("transition:",transition,"full left step")

        self.x_trajectory.initiate_trajectories()
        self.z_trajectory.initiate_trajectories()
        self.x_trajectory.update_trajectory()
        self.z_trajectory.update_trajectory()
        z = self.z_trajectory.f0

        if transition == 5 or transition == 6:
            x = self.x_trajectory.f0
            print("transition:",transition,"half step starting")
        else:
            x = self.x_trajectory.f0 - self.process_parameters[0]/2
            print("transition:",transition,"half step stopping")


        print("step_length:", self.process_parameters[0],"length in trajectory:", self.x_trajectory.step_length)
        print("transition:", transition,"\nx_trajectory: ", x)
        print("transition:", transition,"\nz_trajectory: ", z)
        # input("enter to continue")
        

        hip_pose = Pose()
        hip_pose.position.y = self.ik_solver.l2 + self.ik_solver.l3
        hip_pose.position.x = x[0,0]

        self.command_joint_angles = np.zeros([self.x_trajectory.no_of_trajectory_points,6])
        # print(self.command_joint_angles)
        
        for i in range(self.x_trajectory.no_of_trajectory_points):
            # swing leg calculation
            swing_leg_angles = self.ik_solver.get_angles(x[0,i],hip_pose.position.y- z[0,i])
            # stance leg calculation
            stance_leg_angles =  math.asin(-x[0,i]/(self.ik_solver.l2 + self.ik_solver.l3))
        
            # 1 or 3 swing leg is right leg
            # 2 or 4 swing leg is left leg
            # print(swing_leg_angles[0][1])
            if transition == 2 or transition == 4 or transition == 6:
                self.command_joint_angles[i,0] = 0
                self.command_joint_angles[i,1] = math.degrees(swing_leg_angles[0][0])
                self.command_joint_angles[i,2] = math.degrees(swing_leg_angles[0][1])
                self.command_joint_angles[i,3] = 0
                self.command_joint_angles[i,4] = math.degrees(stance_leg_angles)
                self.command_joint_angles[i,5] = 0
            
            elif transition == 1 or transition == 3 or transition == 5:
                self.command_joint_angles[i,0] = 0
                self.command_joint_angles[i,1] = math.degrees(stance_leg_angles)
                self.command_joint_angles[i,2] = 0
                self.command_joint_angles[i,3] = 0
                self.command_joint_angles[i,4] = math.degrees(swing_leg_angles[0][0])
                self.command_joint_angles[i,5] = math.degrees(swing_leg_angles[0][1])

        print("inside the process", self.command_joint_angles.shape[0])

    def get_process_joint_commands(self):
        print("inside the process joint commands", self.command_joint_angles.shape[0])

        return self.command_joint_angles

    def update_process_parameters(self, parameters):
        self.process_parameters = [parameters[0], parameters[1], parameters[2]]
    
    def reinitialize_trajectories(self):
        self.command_joint_angles = np.zeros([1,6])
    
class SitToStand(Process):
    def __init__(self):
        super().__init__()
        self.curr_states = [0,1]
        self.curr_state = 0
        # 0 for standing
        # 1 for sitting

    def run(self):
        
        if self.end_process:
            if self.curr_state == 1: 
                self.is_at_end = True
        
        else:
            if self.curr_state == 0:
                self.update_trajectory()
                self.update_state()
            else:
                return

    def update_trajecory(self):
        # sitting trajectory here
        self.command_joint_angles = np.zeros([1,6])
        

    def update_state(self):
        if self.curr_state == 0:
            self.curr_state = 1

class Sit(Process):
    def __init__(self):
        super().__init__()
        self.curr_states = [0,1]
        self.transitions = []
        self.curr_state = 0
        # 0 for standing
        # 1 for sitting

    def run(self):
        
        if self.end_process:
            if self.curr_state == 1: 
                print("End process is True, Endding the process")
                print("Already Sitting, ending the process")
                self.is_at_end = True
        else:
            if self.curr_state == 0:
                self.update_trajectory()
                print("Sitting trajectory is published!")
                self.curr_state = 1
            else:
                print("sitting completed")
                return

    def update_trajectory(self):
        if self.curr_state == 1:
            print("Sitting trajectory published")
        pass

