#!/usr/bin/env python3

from exoskeleton_btp.msg import imuData
from sensor_msgs.msg import Imu
import std_msgs.msg
import rospy

robot1ArduinoTopic = 'robot1_imu'
robot2ArduinoTopic = 'robot2_imu'
robot3ArduinoTopic = 'robot3_imu'
robot4ArduinoTopic = 'robot4_imu'

robot1PcTopic = 'robot1_viz_imu'
robot2PcTopic = 'robot2_viz_imu'
robot3PcTopic = 'robot3_viz_imu'
robot4PcTopic = 'robot4_viz_imu'

robot1Imu = Imu()
robot2Imu = Imu()
robot3Imu = Imu()
robot4Imu = Imu()

allRobots = [robot1Imu, robot2Imu, robot3Imu, robot4Imu ]
def initiateMsgs():
    for i in range(len(allRobots)):
        allRobots[i].header = std_msgs.msg.Header
        allRobots[i].header.frame_id = 'robot' + str(i)
        allRobots[i].header.seq = 0
        allRobots[i].header.stamp = rospy.Time.now()

        allRobots[i].angular_velocity.x = 0
        allRobots[i].angular_velocity.y = 0
        allRobots[i].angular_velocity.z = 0

        allRobots[i].linear_acceleration.x = 0
        allRobots[i].linear_acceleration.y = 0
        allRobots[i].linear_acceleration.z = 0

        allRobots[i].angular_velocity_covariance = [0.0 for i in range(9)]
        allRobots[i].linear_acceleration_covariance = [0.0 for i in range(9)]





def robot1Callback(data):
    print("In robot1 calback")
    print(data)
    allRobots[0].angular_velocity.x = data.wx
    allRobots[0].angular_velocity.y = data.wy
    allRobots[0].angular_velocity.z = data.wz

    allRobots[0].linear_acceleration.x = data.ax
    allRobots[0].linear_acceleration.y = data.ay
    allRobots[0].linear_acceleration.z = data.az



def robot2Callback(data):
    print("In robot2 calback")
    print(data)
    allRobots[1].angular_velocity.x = data.wx
    allRobots[1].angular_velocity.y = data.wy
    allRobots[1].angular_velocity.z = data.wz2
    allRobots[1].linear_acceleration.x = data.ax
    allRobots[1].linear_acceleration.y = data.ay
    allRobots[1].linear_acceleration.z = data.az



def robot3Callback(data):
    print("In robot3 calback")
    print(data)
    allRobots[2].angular_velocity.x = data.wx
    allRobots[2].angular_velocity.y = data.wy
    allRobots[2].angular_velocity.z = data.wz
    allRobots[2].linear_acceleration.x = data.ax
    allRobots[2].linear_acceleration.y = data.ay
    allRobots[2].linear_acceleration.z = data.az

def robot4Callback(data):
    print("In robot4 calback")
    print(data)
    allRobots[3].angular_velocity.x = data.wx
    allRobots[3].angular_velocity.y = data.wy
    allRobots[3].angular_velocity.z = data.wz

    allRobots[3].linear_acceleration.x = data.ax
    allRobots[3].linear_acceleration.y = data.ay
    allRobots[3].linear_acceleration.z = data.az

def publishAll():
    robot1Pub.publish(robot1Imu)
    robot2Pub.publish(robot1Imu)
    robot3Pub.publish(robot1Imu)
    robot4Pub.publish(robot1Imu)

if __name__== "__main__" :
    rospy.init_node("imu_listener", anonymous=True)
    robot1Sub = rospy.Subscriber(robot1ArduinoTopic, imuData, robot1Callback)
    robot2Sub = rospy.Subscriber(robot2ArduinoTopic, imuData, robot2Callback)
    robot3Sub = rospy.Subscriber(robot3ArduinoTopic, imuData, robot3Callback)
    robot4Sub = rospy.Subscriber(robot4ArduinoTopic, imuData, robot4Callback)

    robot1Pub = rospy.Publisher(robot1PcTopic, Imu, queue_size=10)
    robot2Pub = rospy.Publisher(robot2PcTopic, Imu, queue_size=10)
    robot3Pub = rospy.Publisher(robot3PcTopic, Imu, queue_size=10)
    robot4Pub = rospy.Publisher(robot4PcTopic, Imu, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publishAll()
        rate.sleep()

    