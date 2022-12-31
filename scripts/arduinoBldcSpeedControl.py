#!/usr/bin/env python3

import rospy
from exoskeleton_btp.msg import exoJointCommand,jointFeedback

joint1 = jointFeedback()
joint1Command = exoJointCommand()
joint1Command.velocity = 0
joint1Command.direction = False
commandPublisher = None
jointFeedbackListener = None

def publisherInit():
    commandTopic = 'pc_joint1Command'
    commandPublisher = rospy.Publisher(commandTopic, exoJointCommand, queue_size=10)
    commandPublisher.publish(joint1Command)

def joint1FeedbackListenerCallback(data):
    joint1.speed = data.speed 
    joint1.direction = data.direction
    joint1.encoderData = data.encoderData
    joint1.hallState = data.hallState 
    print("recieved data is")
    print(joint1)

def subscriberInit():
    jointFeedbackListener = rospy.Subscriber("arduino_joint1Feedback", jointFeedback, joint1FeedbackListenerCallback)

if __name__=='__main__':
    mynode = rospy.init_node("arduino_communicator_pc")
    publisherInit()
    subscriberInit()
    r = rospy.Rate(10)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     print("publishing the message")
    #     commandPublisher.publish(joint1Command)
    #     r.sleep()

    