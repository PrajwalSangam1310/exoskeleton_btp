import math
import numpy as np
from trajectory_generator import x_trajectory, z_trajectory
import matplotlib.pyplot as plt



class BipedInverseKinematics:
    def __init__(self):
        self.joint_angles = np.zeros((1,2))    
        self.l2 = 477.84 * 0.001 # shin length
        self.l3 = 434.74 *0.001 # thigh length
        

    def get_angles(self,x,y):
        arg =(self.l2**2 + self.l3**2 - (y**2 + x**2))/(2*self.l2*self.l3)
        if abs(arg) < 1:
            psi = math.acos(arg) 
        else:
            if arg < -1:
                psi = math.pi
            else:
                psi = 0
        # Sirs slide
        # if x < 0:
        #     x = -x
        #     negative = True
        # print("this is psi",psi)
        # theta2 = math.asin(
        #                     (   y*self.l3*math.sin(psi) + x*(self.l2 + self.l3*math.cos(psi))   )   /
        #                     (   (self.l2 + self.l3*math.cos(psi))**2 + (self.l3*math.sin(psi))**2)    
        #                 )
        # theta3 = theta2 - math.atan2(y,x)
        # print("this is theta2",theta2)

        alpha = math.atan2(x,y)

        # print(math.degrees(alpha))
        arg =(-self.l2**2 + self.l3**2 +(y**2 + x**2))/(2*math.sqrt(y**2 + x**2)*self.l3)
        if abs(arg) < 1:
            beta = math.acos(arg) 
        else:
            if arg < -1:
                beta = math.pi
            else:
                beta = 0

        theta2 = math.pi-psi
        theta3 = alpha + beta
        # print("theta2 is",math.degrees(theta2))

        self.joint_angles[0,0] = theta3
        self.joint_angles[0,1] = theta2
        # if negative:
        #     self.joint_angles[0,0] = -theta3
        return self.joint_angles

if __name__ == '__main__':
    x = x_trajectory()
    x.initiate_trajectories()
    x.step_length = 0.1
    x.update_trajectory()

    z = z_trajectory()
    z.initiate_trajectories()
    z.step_height = 0.2
    z.update_trajectory()



    ik_solver = BipedInverseKinematics()
    hip_y = 0.9
    l2 = 0.5
    l3 = 0.5

    angles = ik_solver.get_angles(0,0.9)
    print(angles[0,0]*180/3.14, angles[0,1]*180/3.14)
    exit()
    for i in range(x.f0.size):
        # angles = ik_solver.get_angles(x.f1[0,i]-hip_pose.position.x,hip_pose.position.y - z.f0[0,i])
        x1,y1 = 0,0.9
        angles = ik_solver.get_angles(x.f0[0,i],hip_y - z.f0[0,i])
        theta3 = angles[0,0]
        theta2 = angles[0,1]

        x2,y2 = x1 + l3*math.sin(theta3), y1 - l3*math.cos(theta3)
        x3,y3 = x2 - l2*math.sin(theta2), y2 - l2*math.cos(theta2)
        plt.plot([x1,x2,x3],[y1,y2,y3])
    plt.plot(x.f0[0], z.f0[0])    
    plt.show()
    




