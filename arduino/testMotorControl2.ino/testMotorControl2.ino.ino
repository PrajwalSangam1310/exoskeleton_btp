#include <ros.h>
#include <exoskeleton_btp/velocityEffort.h>
#include <exoskeleton_btp/hallState.h>
#include <IFX007T-Motor-Control.h>

// ---------------------------------------------------------------------
#define DIRECTION  0           // 0 or 1
#define WEAKENING  0           // 0=normal, 1=weak (fast)
// ---------------------------------------------------------------------

uint16_t rpmSpeed = 0;
uint16_t rpmSpeedRos = 0;

//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();
bool direction;
bool rosDirection = false;

//Initialize ros part
bool useRos = false;

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
ros::Subscriber<exoskeleton_btp::velocityEffort> commandVelocitySubscriber("arduino_joint1Velocity_command", commandVelocityCb);
void setup()
{
//  serial_init();
  ros_init();
//  motor_init();

}

void serial_init() {
//  Serial.begin(115200);
  Serial.println(" Infineon hall sensor BLDC motor test! ");

}

void ros_init() {
//  Serial.begin(115200);
//  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(hallStatePublisher);
  //  nh.subscribe(commandVelocitySubscriber);
}

void motor_init() {
  MyMotor.begin();
//  Adapt the following values according to the README if necessary
  MyMotor.MotorParam.MotorPolepairs = 4;        // Amount of polepairs. If your motor has 8 poles, it has 4 pole PAIRS
  MyMotor.MotorParam.SensingMode = 1;           // If you use a Hallsensor set 1, for sensorless application 0
  MyMotor.MotorParam.PI_Reg_P = 0.01;           // P value for the PI-RPM regulator
  MyMotor.MotorParam.PI_Reg_I = 0.01;          // I value for the PI-RPM regulator
  
  MyMotor.configureBLDCMotor(MyMotor.MotorParam);
}

void loop()
{
  ros_loop();
}

void ros_loop() {
  //  ros_motor();
  //  joint1HallState.state = MyMotor.getHallState();
  joint1HallState.state = 1;
  hallStatePublisher.publish(&joint1HallState);
  delay(100);
}

void ros_motor() {
  if (useRos) {
    if (rosDirection)
      MyMotor.setHallBLDCmotorRPMspeed(1, rpmSpeedRos, WEAKENING);
    else
      MyMotor.setHallBLDCmotorRPMspeed(0, rpmSpeedRos, WEAKENING);

  }
  else {
    MyMotor.setHallBLDCmotorRPMspeed(1, rpmSpeed, WEAKENING);
  }

}

void motor_loop() {
  if (Serial.available() > 0)
  {
    uint8_t in = Serial.read();
    if (in == '+') rpmSpeed += 200;         // Adapt the speed with keyboard input in the serial monitor
    if (in == '-') rpmSpeed -= 200;
    Serial.println(rpmSpeed);
  }
}
