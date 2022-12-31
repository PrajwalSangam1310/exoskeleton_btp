#!/usr/bin/env python3

from sympy import euler
from tinyik import Joint
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import broadcaster
from exoskeleton_btp.msg import imuQuaternion 
import copy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import yaml 

def cb(data):
    imu_datas = data

class visualizeImu:
    def __init__(self):
        self.joint_names =     ["joint_left_abd"
                                ,"joint_left_hip_flexion"
                                ,"joint_left_knee_flexion"
                                ,"joint_left_ankle_flexion"
                                ,"joint_right_abd"
                                ,"joint_right_hip_flexion"
                                ,"right_knee_joint"
                                ,"right_ankle_flexion"]
        self.no_of_joints = 8
        self.joint_states = JointState()
        self.joint_state_init()
        self.thetas = [0.0,0.0]

        self.imu_datas = imuQuaternion()
        self.imu_subscriber = rospy.Subscriber("/exoskeleton_btp/imuDatas", imuQuaternion, self.imu_cb)
        self.joint_state_publisher = rospy.Publisher("/joint_states", JointState, queue_size=10)

    def joint_state_init(self):
        self.joint_states.name = self.joint_names
        self.joint_states.position = [0.0 for i in range(self.no_of_joints)]
        self.joint_states.header = Header()
        self.joint_states.header.frame_id = "base_link"
        self.joint_states.header.stamp = rospy.Time.now()
    
    def update_joint_positions(self):
        for i in range(len(self.imu_datas.datas)):
            eulers = euler_from_quaternion([ self.imu_datas.datas[i].x, self.imu_datas.datas[i].y, self.imu_datas.datas[i].z, self.imu_datas.datas[i].w])
            # eulers = euler_from_quaternion(self.imu_datas.datas[i])
            self.thetas[i] = eulers[2]
            # euler_from_quaternion()
        
        self.joint_states.position[1] = self.thetas[0]
        self.joint_states.position[2] = self.thetas[1] - self.thetas[0]
    
    def imu_cb(self, data):
        self.imu_datas = data
        self.update_joint_positions()
    
    def show_thetas(self):
        print("Hip flexion:", self.thetas[0], "Knee flexion:", self.thetas[1])

    def publish(self):
        self.joint_states.header.stamp =  rospy.Time.now()
        self.joint_state_publisher.publish(self.joint_states)

if __name__ =="__main__":
    rospy.init_node("leg_visualizer", anonymous=True)
    temp = visualizeImu()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        temp.publish()
