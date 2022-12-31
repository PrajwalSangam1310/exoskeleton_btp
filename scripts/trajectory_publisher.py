#!/usr/bin/env python3

from multiprocessing.connection import Listener
from tkinter import X
import rospy
import tf
from trajectory_generator import x_trajectory, z_trajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Transform, Pose
from colorama import Fore
from std_msgs.msg import Header

def get_hip_location():
    pass

def get_ankle_location():
    pass

class DisplayTrajectory:
    def __init__(self):
        self.Rtraj = Path()
        self.Ltraj = Path()
        self.world_frame_name = "world"
        self.left_topic_name = "exoskeleton/LeftAnkleTraj"
        self.right_topic_name = "exoskeleton/RightAnkleTraj"
        self.initiate_traj()
        
        self.LeftAnklePose = Pose()
        self.RightAnklePose = Pose()

        self.publisher_init()


    def initiate_traj(self):
        self.Rtraj.header = Header()
        self.Rtraj.header.frame_id = self.world_frame_name 
        self.Rtraj.header.stamp = rospy.Time()
        self.Rtraj.header.seq = 0
        self.Rtraj.poses = []

        self.Ltraj.header = Header()
        self.Ltraj.header.frame_id = self.world_frame_name 
        self.Ltraj.header.stamp = rospy.Time()
        self.Ltraj.header.seq = 0
        self.Ltraj.poses = []

        self.publisher_init()

    def publisher_init(self):
        print(self.left_topic_name)
        self.l_traj_publisher = rospy.Publisher(self.left_topic_name, Path, queue_size= 10)
        self.r_traj_publisher = rospy.Publisher(self.right_topic_name, Path, queue_size= 10)

    def update_trajectories(self, xL, zL, xR, zR):
        # for left leg
        self.Ltraj.poses = [PoseStamped() for i in range(xL.size)]
        for i in range(xL.size):
            # defining the header for pose stamped
            self.Ltraj.poses[i].header.frame_id = self.world_frame_name 
            self.Ltraj.poses[i].header.seq = 0
            self.Ltraj.poses[i].header.stamp = rospy.Time()

            # setting the poses with respect to the world frame 
            self.Ltraj.poses[i].pose = Pose()
            self.Ltraj.poses[i].pose.position.x = xL[0,i] + self.LeftAnklePose.position.x/2
            self.Ltraj.poses[i].pose.position.y = self.LeftAnklePose.position.y
            self.Ltraj.poses[i].pose.position.z = zL[0,i] + self.LeftAnklePose.position.z
            self.Ltraj.poses[i].pose.orientation.w = 1

        #for right leg
        self.Rtraj.poses = [PoseStamped() for i in range(xR.size)]
        for i in range(xR.size):
            # defining the header for pose stamped
            self.Rtraj.poses[i].header.frame_id = self.world_frame_name 
            self.Rtraj.poses[i].header.seq = 0
            self.Rtraj.poses[i].header.stamp = rospy.Time()

            # setting the poses with respect to the world frame 
            self.Rtraj.poses[i].pose = Pose()
            self.Rtraj.poses[i].pose.position.x = xR[0,i] + self.RightAnklePose.position.x
            self.Rtraj.poses[i].pose.position.y = self.RightAnklePose.position.y
            self.Rtraj.poses[i].pose.position.z = zR[0,i] + self.RightAnklePose.position.z
            self.Rtraj.poses[i].pose.orientation.w = 1        

    def publish_trajs(self):
        self.Ltraj.header.stamp = rospy.Time()
        self.Rtraj.header.stamp = rospy.Time()
        self.l_traj_publisher.publish(self.Ltraj)
        self.r_traj_publisher.publish(self.Rtraj)         

    def update_ankle_positions(self, left_ankle_position, right_ankle_position):
        self.LeftAnklePose.position.x = left_ankle_position[0]
        self.LeftAnklePose.position.y = left_ankle_position[1]
        self.LeftAnklePose.position.z = left_ankle_position[2]

        self.RightAnklePose.position.x = right_ankle_position[0]
        self.RightAnklePose.position.y = right_ankle_position[1]
        self.RightAnklePose.position.z = right_ankle_position[2]



if __name__ == '__main__':
    rospy.init_node("Trajectory_publisher",anonymous=True)

    listener = tf.TransformListener()

    x = x_trajectory()
    x.initiate_trajectories()
    x.update_trajectory()

    z = z_trajectory()
    z.initiate_trajectories()
    z.update_trajectory()

    display_trajectory = DisplayTrajectory()

    while not rospy.is_shutdown():
        try:
            (trans_r,rot_r) = listener.lookupTransform('world', 'left_foot', rospy.Time(0))
            (trans_l,rot_l) = listener.lookupTransform('world', 'right_foot', rospy.Time(0))
            (trans_h,rot_h) = listener.lookupTransform('world', 'hip', rospy.Time(0))

            display_trajectory.update_ankle_positions(trans_l, trans_r)
            print(Fore.GREEN+"Got_ankle_posiitons, updating..."+Fore.RESET)
            break
            # print(type(transR), transR, rotR)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print(Fore.RED +"Master Prajwal, An exception occured during! " + Fore.RESET)
            continue
    
    display_trajectory.update_trajectories(x.f0,z.f0,x.f0,z.f0)
    print(Fore.BLUE+"Trajectories updated!!!"+Fore.RESET)
    display_trajectory.publish_trajs()
    print(Fore.GREEN+"Trajectories published!!!"+Fore.RESET)


        


        



        
    
        
    