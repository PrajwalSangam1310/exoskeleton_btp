#!/usr/bin/env python3
import rospy
from exoskeleton_btp.msg import exoAllJointFeedbackMsg, exoAllJointCommand, exoAllPid, exoAllJointTrajectory, userInput, uiDisplayMessage
from paho.mqtt.client import Client
import struct

class ExoRosMqttEndPoint:
    def __init__(self):
        self.exo_mqtt_client  = Client()
        self.no_of_motors = 6
        self.joint_names = ["right_abduction", "right_flexion", "right_knee", "left_abduction", "left_flexion", "left_knee"]
        
        self.mqtt_feedback_topics = ['/RIGHT_ABDUCTION/feedback_data','/RIGHT_FLEXION/feedback_data','/RIGHT_KNEE/feedback_data', '/LEFT_ABDUCTION/feedback_data','/LEFT_FLEXION/feedback_data','/LEFT_KNEE/feedback_data']
        self.mqtt_command_topics = ['/RIGHT_ABDUCTION/command_data','/RIGHT_FLEXION/command_data', '/RIGHT_KNEE/command_data', '/LEFT_ABDUCTION/command_data','/LEFT_FLEXION/command_data', '/LEFT_KNEE/command_data']
        self.mqtt_pid_topics = ['/RIGHT_ABDUCTION/pid','/RIGHT_FLEXION/pid', '/RIGHT_KNEE/pid', '/LEFT_ABDUCTION/pid','/LEFT_FLEXION/pid', '/LEFT_KNEE/pid']

        ## ui topics
        self.uiDisplayTopic = 'exo/display'
        self.userInputTopic = 'exo/userInput'

        self.ros_feedback_topic = 'exo/feedback_data'
        self.ros_command_topic = 'exo/command_data'
        self.ros_pid_topic = 'exo/pid'
        self.ros_trajectory_topic = 'exo/trajectory'

        self.ros_feedback_publisher = rospy.Publisher(self.ros_feedback_topic, exoAllJointFeedbackMsg, queue_size=10)
        self.ros_command_subscriber = rospy.Subscriber(self.ros_command_topic, exoAllJointCommand, self.command_cb)
        self.ros_pid_subscriber = rospy.Subscriber(self.ros_pid_topic, exoAllPid, self.pid_cb)
        self.ros_trajectory_subscriber = rospy.Subscriber(self.ros_trajectory_topic, exoAllJointTrajectory, self.trajectory_callback)
        # self.ui_display_subscriber = rospy.Subscriber(self.uiDisplayTopic, uiDisplayMessage, self.display_cb)


        self.ros_feedback_message = exoAllJointFeedbackMsg()
        self.ros_feedback_message.joint_names = self.joint_names
        self.ros_pid_message = exoAllPid()
        self.ros_pid_message.joint_names = self.joint_names
        self.ros_command_message = exoAllJointCommand()

        self.ros_command_message.joint_names = self.joint_names

        #battery levels
        self.battery_percentage = [100, 95, 90, 85, 80, 75, 70, 65, 60 , 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0]
        self.battery_voltages  = [4.2, 4.15, 4.11, 4.08, 4.02, 3.98, 3.95, 3.91, 3.87, 3.85, 3.84, 3.82, 3.8, 3.79, 3.77, 3.75, 3.73, 3.71, 3.69, 3.61, 3.27 ] 

        #ui messages
        self.user_input_message = userInput()
        self.ros_userinput_publisher = rospy.Publisher(self.userInputTopic, userInput, queue_size=10)
        self.display_message = uiDisplayMessage()


        self.feedback_struct = struct.Struct('<10f')
        self.command_struct = struct.Struct('<3f2i')
        self.pid_struct = struct.Struct('<3f')

        ## ui structs
        self.ui_display_struct = struct.Struct('<6f6i')
        self.user_input_struct = struct.Struct('3f2B')

        self.mqtt_init()

    def get_battery_percentage(self, index):
        voltages = [self.ros_feedback_message.battery_level[index].cell1,
                    self.ros_feedback_message.battery_level[index].cell2,
                    self.ros_feedback_message.battery_level[index].cell3]
        min_voltage = min(voltages)
        for i in range(len(self.battery_percentage)):
            if min_voltage > self.battery_voltages[i]:
                return self.battery_percentage[i]
        return 0



    def mqtt_init(self):
        self.host = "192.168.187.251"
        self.exo_mqtt_client.on_message = self.mqtt_on_message
        self.exo_mqtt_client.on_connect = self.mqtt_on_connect
        self.exo_mqtt_client.connect(self.host, 1883)
        self.exo_mqtt_client.loop_start()
        for i in range(self.no_of_motors):
            self.exo_mqtt_client.subscribe(self.mqtt_feedback_topics[i])
            self.exo_mqtt_client.message_callback_add(self.mqtt_feedback_topics[i], self.mqtt_feedback_callback)
        
        self.exo_mqtt_client.subscribe(self.userInputTopic)
        self.exo_mqtt_client.message_callback_add(self.userInputTopic, self.user_input_cb)

    def user_input_cb(self, client, userdata,message):
        received_topic  = message.topic
        qos = message.qos
        print("received messages")
        unpacked_data = self.user_input_struct.unpack(message.payload)
        print(unpacked_data, received_topic, message.qos)  
        self.user_input_message.walking_speed = unpacked_data[0]
        self.user_input_message.stride_length = unpacked_data[1]
        self.user_input_message.step_height = unpacked_data[2]
        self.user_input_message.action_selected = unpacked_data[3]
        self.user_input_message.is_on = unpacked_data[4]
        
        self.ros_userinput_publisher.publish(self.user_input_message)

    def mqtt_on_message(self, client, userdata, message):
        received_topic  = message.topic
        qos = message.qos
        print("received messages")
        unpacked_data = self.feedback_struct.unpack(message.payload)
        print(unpacked_data, received_topic, message.qos)

    def mqtt_on_connect(self, client, userdata,flags, rc):
        print("Connected to the mqtt server")

    #mqtt callback
    def mqtt_feedback_callback(self, client, userdata, message):
        unpacked_data = self.feedback_struct.unpack(message.payload) 
        self.update_feedback_messages(message.topic, unpacked_data)
        print(unpacked_data, message.topic)

    #ros callback
    def command_cb(self, data):
        for i in range(self.no_of_motors):
            self.ros_command_message.goal_position[i] = data.goal_position[i]
            self.ros_command_message.goal_velocity[i] = data.goal_velocity[i]
            self.ros_command_message.goal_effort[i] = data.goal_effort[i]
            self.ros_command_message.control_mode[i] = data.control_mode[i]
            self.ros_command_message.interrupt[i] = data.interrupt[i]
            packed_data = self.command_struct.pack(data.goal_position[i], data.goal_velocity[i], data.goal_effort[i], data.control_mode[i], data.interrupt[i])
            self.exo_mqtt_client.publish(self.mqtt_command_topics[i], packed_data)
        print(self.ros_command_message)
        
    #pid ros callback
    def pid_cb(self, data):
        for i in range(self.no_of_motors):
            self.ros_pid_message.p[i] = data.p[i]
            self.ros_pid_message.i[i] = data.i[i]
            self.ros_pid_message.d[i] = data.d[i]
            packed_data = self.pid_struct.pack(data.p[i], data.i[i], data.d[i])
            self.exo_mqtt_client.publish(self.mqtt_pid_topics[i], packed_data)
        print(self.ros_pid_message)

    def update_feedback_messages(self, topic_name, unpacked_data):
        i = self.mqtt_feedback_topics.index(topic_name)
        print(topic_name, i)
        self.ros_feedback_message.position[i] = unpacked_data[0]
        self.ros_feedback_message.velocity[i] = unpacked_data[1]

        self.ros_feedback_message.battery_level[i].cell1 = unpacked_data[2]
        self.ros_feedback_message.battery_level[i].cell2 = unpacked_data[3]
        self.ros_feedback_message.battery_level[i].cell3 = unpacked_data[4]

        
        self.ros_feedback_message.motor_current[i] = unpacked_data[5]
        self.ros_feedback_message.imu[i].x = unpacked_data[6]
        self.ros_feedback_message.imu[i].y = unpacked_data[7]
        self.ros_feedback_message.imu[i].z = unpacked_data[8]
        self.ros_feedback_message.imu[i].w = unpacked_data[9]
        self.ros_feedback_publisher.publish(self.ros_feedback_message)
        
        # add the battery status in the display message which should be published through the mqtt network
        self.display_message.battery_level[i] = self.get_battery_percentage(i)
        self.mqtt_publish_display_message()

    def mqtt_publish_display_message(self):
        packed_data = self.ui_display_struct.pack(
            self.display_message.battery_level[0],
            self.display_message.battery_level[1],
            self.display_message.battery_level[2],
            self.display_message.battery_level[3],
            self.display_message.battery_level[4],
            self.display_message.battery_level[5],
            self.display_message.connection_status[0],
            self.display_message.connection_status[1],
            self.display_message.connection_status[2],
            self.display_message.connection_status[3],
            self.display_message.connection_status[4],
            self.display_message.connection_status[5])

        self.exo_mqtt_client.publish(self.uiDisplayTopic, packed_data)

    #trajectory callback
    def trajectory_callback(self, data):
        for i in range(self.no_of_motors):
            self.ros_command_message.goal_position[i] = data.goal_position[i]
            self.ros_command_message.goal_velocity[i] = data.goal_velocity[i]
            packed_data = self.command_struct.pack(data.goal_position[i], data.goal_velocity[i], self.ros_command_message.goal_effort[i], self.ros_command_message.control_mode[i], self.ros_command_message.interrupt[i])
            self.exo_mqtt_client.publish(self.mqtt_command_topics[i], packed_data)
        print(self.ros_command_message)

    
        

if __name__ == "__main__":
    rospy.init_node("RosMqttEndpoint", anonymous = True)
    endpoint = ExoRosMqttEndPoint()
    rospy.spin()
    endpoint.exo_mqtt_client.loop_stop()


    

