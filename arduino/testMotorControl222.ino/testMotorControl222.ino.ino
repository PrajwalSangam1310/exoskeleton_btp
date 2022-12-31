#include <ros.h>
#include <exoskeleton_btp/velocityEffort.h>
#include <exoskeleton_btp/hallState.h>

// ---------------------------------------------------------------------
#define DIRECTION  0           // 0 or 1
#define WEAKENING  0           // 0=normal, 1=weak (fast)
// ---------------------------------------------------------------------
//Initialize ros part
bool useRos = false;


//temp
uint16_t rpmSpeedRos;

//create node
ros::NodeHandle nh;

// create the msgs
exoskeleton_btp::hallState joint1HallState;

//create the publishers
ros::Publisher hallStatePublisher("arduino_joint1HallState", &joint1HallState);


//callback
//exoskeleton_btp::velocityEffort rosRefVelocityEffort;


void commandVelocityCb(const exoskeleton_btp::velocityEffort& tempCommand) {
  rpmSpeedRos = tempCommand.velocity;
}

//command subscriber
//ros::Subscriber<exoskeleton_btp::velocityEffort> commandVelocitySubscriber("arduino_joint1Velocity_command", commandVelocityCb);
void setup()
{
//  serial_init();
  ros_init();
//  motor_init();

}

void serial_init() {
  Serial.begin(115200);
  Serial.println(" Infineon hall sensor BLDC motor test! ");

}

void ros_init() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
//  nh.advertise(hallStatePublisher);
  //  nh.subscribe(commandVelocitySubscriber);
}

void loop()
{
  ros_loop();
}

void ros_loop() {
  //  ros_motor();
  //  joint1HallState.state = MyMotor.getHallState();
  joint1HallState.state = 1;
//  hallStatePublisher.publish(&joint1HallState);
}
