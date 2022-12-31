#!/usr/env/bin python3

import numpy as np
import matplotlib.pyplot as plt

class trajectory:
    def __init__(self):        
        #change the stop time accordingly.
        self.stop_time = 1

        # no of trajectory points in the trajectory.
        self.no_of_trajectory_points = 100

    def init_params(self):
        #initialize the parameters that will be used in your trajectory
        # Example :
        #   self.step_length = 0.5
        #   self.step_time = 3
        #   self.no_of_trajectory_points = 100
        return

    def initiate_trajectories(self):
        self.f0 = np.zeros((1,self.no_of_trajectory_points))
        self.f1 = np.zeros((1,self.no_of_trajectory_points))
        self.f2 = np.zeros((1,self.no_of_trajectory_points))
        self.ts = np.array(np.linspace(0,self.stop_time, self.no_of_trajectory_points))
        self.ts.shape = (1,self.no_of_trajectory_points)

    def update_trajectory(self):
        for i in range(self.f0.size):
            self.f0[0,i] = self.f0_equation(self.ts[0,i])
            self.f1[0,i] = self.f1_equation(self.ts[0,i])
            self.f2[0,i] = self.f2_equation(self.ts[0,i])

    def f0_equation(self,t):
        #define the trajectory here
        return 0
        return 3*self.step_length*t**2/(self.step_time**2)\
            -2*self.step_length*t**3/(self.step_time**3)

    def f1_equation(self,t):
        #define the trajectory here
        return 0
        return 6*self.step_length*t/(self.step_time**2)\
            -6*self.step_length*t**2/(self.step_time**3)

    def f2_equation(self,t):
        #define the trajectory here
        return 0
        return 6*self.step_length/(self.step_time**2)\
            -12*self.step_length*t/(self.step_time**3)

    def show_trajectories(self):
        plt.plot(self.ts[0,:],self.f0[0,:],label='f0')
        plt.plot(self.ts[0,:],self.f1[0,:],label='f1')
        plt.plot(self.ts[0,:],self.f2[0,:],label='f2')
        plt.legend()
        plt.show()

class x_trajectory(trajectory):
    def __init__(self):
        super().__init__()
        self.step_length = 0.5
        self.step_time = 3
        self.stop_time = self.step_time

    def update_params(self, step_time, step_length):
        self.step_time = step_time
        self.step_length = step_length
        self.stop_time = self.step_time

    def f0_equation(self,t):
        return 3*self.step_length*t**2/(self.step_time**2)\
            -2*self.step_length*t**3/(self.step_time**3)

    def f1_equation(self,t):
        return 6*self.step_length*t/(self.step_time**2)\
            -6*self.step_length*t**2/(self.step_time**3)

    def f2_equation(self,t):
        return 6*self.step_length/(self.step_time**2)\
            -12*self.step_length*t/(self.step_time**3)

class z_trajectory(trajectory):
    def __init__(self):
        super().__init__()
        self.step_height = 0.1
        self.step_time = 3
        self.stop_time = self.step_time

    def update_params(self, step_time, step_height):
        self.step_time = step_time
        self.step_height = step_height
        self.stop_time = self.step_time

    def f0_equation(self,t):
        return 16*self.step_height*t**2/(self.step_time**2)\
            -32*self.step_height*t**3/(self.step_time**3)\
                +16*self.step_height*t**4/self.stop_time**4

    def f1_equation(self,t):
        return 32*self.step_height*t/(self.step_time**2)\
            -96*self.step_height*t**2/(self.step_time**3)\
                + 64*self.step_height*t**3/self.step_time**4

    def f2_equation(self,t):
        return 32*self.step_height/(self.step_time**2)\
            -192*self.step_height*t/(self.step_time**3)\
            + 192*self.step_height*t**2/self.step_time**4
        
    
            
if __name__ == '__main__':
    x = x_trajectory()
    x.initiate_trajectories()

    x.update_trajectory()
    x.show_trajectories()

    z = z_trajectory()
    z.initiate_trajectories()

    z.update_trajectory()
    z.show_trajectories()




    

