#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Transform, Pose
from gazebo_msgs.msg import ModelStates

base_link_pose = Pose()


br = tf.TransformBroadcaster()


base_to_hip = Transform()
base_to_hip.translation.x = 0.1866
base_to_hip.translation.y = -0.293825
base_to_hip.translation.z = 0.14796


def modelStatesCb(data):
    model_name = "robot"
    i = data.name.index(model_name)
    base_link_pose.position.x = data.pose[i].position.x 
    base_link_pose.position.y = data.pose[i].position.y 
    base_link_pose.position.z = data.pose[i].position.z 
    base_link_pose.orientation = data.pose[i].orientation
    
    br.sendTransform((base_to_hip.translation.x, base_to_hip.translation.y, base_to_hip.translation.z),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "hip",
                     "base_link")

    br.sendTransform((base_link_pose.position.x, base_link_pose.position.y, base_link_pose.position.z),
                     (base_link_pose.orientation.x, base_link_pose.orientation.y, base_link_pose.orientation.z, base_link_pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "world")

if __name__ == '__main__':
    rospy.init_node("world_to_base_link_tf", anonymous= True)
    sub = rospy.Subscriber("/gazebo/model_states", ModelStates, modelStatesCb)
    rospy.spin()
    


