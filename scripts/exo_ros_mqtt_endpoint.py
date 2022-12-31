#!/usr/bin/env python3
import rospy
from exoskeleton_btp.msg import exoJointFeedbackMsg, exoJointCommand, exoPid
from paho.mqtt.client import Client
import struct

class ExoRosMqttEndPoint:
    def __init__(self):
        self.exo_mqtt_client  = Client()
        self.no_of_motors = 2
        self.feedback_topics = ['/RIGHT_THIGH/feedback_data','/RIGHT_KNEE/feedback_data']
        self.command_topics = ['/RIGHT_THIGH/command_data', '/RIGHT_KNEE/command_data']
        self.pid_topic = ['/RIGHT_THIGH/pid', '/RIGHT_KNEE/pid']

        self.ros_command_subscriber_callbacks = [self.ros_right_thigh_command_callback,
                                            self.ros_right_knee_command_callback]

        self.feedbackPublishers = []
        self.commandSubscribers = []
        self.pidSubscribers = []
        self.ros_feedback_messages = []
        self.ros_pid_messages = []

        for i in range(self.no_of_motors):
            self.feedbackPublishers.append(rospy.Publisher(self.feedback_topics[i], exoJointFeedbackMsg,queue_size= 10))
            self.commandSubscribers.append(rospy.Subscriber(self.command_topics[i], exoJointCommand,self.ros_command_subscriber_callbacks[i]))
            self.pidSubscribers.append(rospy.Subscriber(self.pid_topic, exoPid, self.ros_right_thigh_pid_callback))
            self.ros_feedback_messages.append(exoJointFeedbackMsg())
            self.ros_pid_messages.append(exoPid())

        self.feedback_struct = struct.Struct('<8f')
        self.command_struct = struct.Struct('<3f2i')
        self.pid_struct = struct.Struct('<3f')

        self.mqtt_init()

    def mqtt_init(self):
        self.host = "192.168.34.251"
        self.exo_mqtt_client.on_message = self.mqtt_on_message
        self.exo_mqtt_client.on_connect = self.mqtt_on_connect
        self.exo_mqtt_client.connect(self.host, 1883)
        self.exo_mqtt_client.loop_start()
        for i in range(self.no_of_motors):
            self.exo_mqtt_client.subscribe(self.feedback_topic[i])
            self.exo_mqtt_client.message_callback_add(self.feedback_topic[i], self.mqtt_right_thigh_callback)

    def mqtt_on_message(self, client, userdata, message):
        received_topic  = message.topic
        qos = message.qos
        print("received messages")
        unpacked_data = self.feedback_struct.unpack(message.payload)
        print(unpacked_data, received_topic, message.qos)

    def mqtt_on_connect(self, client, userdata,flags, rc):
        print("Connected to the mqtt server")


    def mqtt_right_thigh_callback(self, client, userdata, message):
        unpacked_data = self.feedback_struct.unpack(message.payload) 
        print(unpacked_data, message.topic, message.qos)
        #convert to ros message and publish

    def update_feedback_messages(self, topic_name, unpacked_data):
        i = self.feedback_topics.index(topic_name)
        print(topic_name, i)
        self.ros_feedback_messages[i].position = unpacked_data[0]
        self.ros_feedback_messages[i].velocity = unpacked_data[1]
        self.ros_feedback_messages[i].battery_level = unpacked_data[2]
        self.ros_feedback_messages[i].motor_current = unpacked_data[3]
        self.ros_feedback_messages[i].imu.x = unpacked_data[4]
        self.ros_feedback_messages[i].imu.y = unpacked_data[5]
        self.ros_feedback_messages[i].imu.z = unpacked_data[6]
        self.ros_feedback_messages[i].imu.w = unpacked_data[7]
        self.feedbackPublishers[i].publish(self.ros_feedback_message)

    def ros_right_thigh_command_callback(self,data):
        self.mqtt_command_publish(self.command_topics, data)

    def ros_right_thigh_pid_callback(self, data):
        self.mqtt_pid_publish(self.pid_topic, data)

    def ros_right_knee_command_callback(self,data):
        self.mqtt_command_publish(self.command_topics, data)

    def ros_right_knee_pid_callback(self, data):
        self.mqtt_pid_publish(self.pid_topic, data)

    def mqtt_command_publish(self, topic, data):
        packed_data = self.command_struct.pack(data.goal_position, data.goal_velocity, data.goal_effort, data.control_mode, data.inturrpt)
        self.exo_mqtt_client.publish(topic, packed_data)
        print("published on the mqtt server")

    def mqtt_pid_publish(self, topic, data):
        packed_data = self.pid_struct.pack(data.p, data.i, data.d)
        self.exo_mqtt_client.publish(topic, packed_data)
        print("published on the mqtt server")

if __name__ == "__main__":
    rospy.init_node("RosMqttEndpoint", anonymous = True)
    endpoint = ExoRosMqttEndPoint()
    rospy.spin()
    endpoint.exo_mqtt_client.loop_stop()


    

