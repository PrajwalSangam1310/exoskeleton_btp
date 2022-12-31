#!/usr/bin/env python3

from calendar import c
import rospy
from exo_execution_handler import ProcessHandler
from exoskeleton_btp.msg import userInput, exoAllJointTrajectory
import numpy as np

def userInputCb(data):
    user_input_message.stride_length = data.stride_length
    user_input_message.walking_speed = data.walking_speed
    user_input_message.action_selected = data.action_selected
    user_input_message.step_height = data.step_height
    user_input_message.is_on = data.is_on
    print("inside userinput callback")
    execution_handler.update_req_id(user_input_message.action_selected)
    execution_handler.processes[1].update_process_parameters([user_input_message.stride_length, user_input_message.walking_speed, user_input_message.step_height])
    # execution_handler.update_process()

if __name__ == "__main__":
    rospy.init_node("process_handler", anonymous=True)
    print("node initiated")
    user_input_topic = "exo/userInput"
    user_input_message = userInput()    
    userInputSub = rospy.Subscriber(user_input_topic, userInput, userInputCb)

    trajectory_topic = "exo/trajectory"
    trajectory_publisher = rospy.Publisher(trajectory_topic, exoAllJointTrajectory, queue_size = 10)
    trajectory_message = exoAllJointTrajectory()

    execution_handler = ProcessHandler()
    publish_rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        execution_handler.update_process()
        joint_angle_commands = execution_handler.processes[execution_handler.curr_process_id].get_process_joint_commands()
        # print(joint_angle_commands)
        for i in range(joint_angle_commands.shape[0]):
            for j in range(6):
                trajectory_message.goal_position[j] = joint_angle_commands[i,j]
            print(i,":", joint_angle_commands[i])
            
            trajectory_publisher.publish(trajectory_message)
            publish_rate.sleep()

            


    


