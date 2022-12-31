#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <exoskeleton_btp/imuData.h>


ros::NodeHandle nh;

// create the publisher

// create the msgs
exoskeleton_btp::imuData robot1Imu;
exoskeleton_btp::imuData robot2Imu;
exoskeleton_btp::imuData robot3Imu;
exoskeleton_btp::imuData robot4Imu;

//create the publishers
ros::Publisher robot1Pub("robot1_imu", &robot1Imu);
ros::Publisher robot2Pub("robot2_imu", &robot2Imu);
ros::Publisher robot3Pub("robot3_imu", &robot3Imu);
ros::Publisher robot4Pub("robot4_imu", &robot4Imu);

// ros::Publisher allImuPublisher("robot1_imu", )


// create the subscriber to recieve the command

void setup() {
  nh.initNode();
  //advertise all the topics
  nh.advertise(robot1Pub);
  nh.advertise(robot2Pub);
  nh.advertise(robot3Pub);
  nh.advertise(robot4Pub);

  robot1Imu.wx = 1;
  robot1Imu.wy = 1;
  robot1Imu.wz = 1;
  robot1Imu.ax = 1;
  robot1Imu.ay = 1;
  robot1Imu.az = 1;
}

void loop() {
  robot1Pub.publish(&robot1Imu);
  nh.spinOnce();
  delay(500);
}
