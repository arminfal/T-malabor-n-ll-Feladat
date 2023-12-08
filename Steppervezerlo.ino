// Include the Arduino Stepper.h library:
#include <AccelStepper.h> //Include the AccelStepper library
#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
// Define the motor pins:
#define MP1  D1 // IN1 on the ULN2003
#define MP2  D2 // IN2 on the ULN2003
#define MP3  D3 // IN3 on the ULN2003
#define MP4  D4 // IN4 on the ULN2003
const int SPR = 4096;//Steps per revolution
#define MotorInterfaceType 8 // Define the interface type as 8 = 4 wires * step factor (2 for half step)
AccelStepper stepper = AccelStepper(MotorInterfaceType, MP1, MP3, MP2, MP4);//Define the pin sequence (IN1-IN3-IN2-IN4)

ros::NodeHandle nh;


void messageCb(const geometry_msgs::Pose& msg){
  Serial.print("Received Quaternion: ");
  Serial.print("x: ");
  Serial.print(msg.orientation.x);
  Serial.print(", y: ");
  Serial.print(msg.orientation.y);
  Serial.print(", z: ");
  Serial.print(msg.orientation.z);
  Serial.print(", w: ");
  Serial.println(msg.orientation.w);

  double yaw = atan2(2.0*(msg.orientation.w*msg.orientation.z + msg.orientation.x*msg.orientation.y), 1.0 - 2.0*(msg.orientation.y*msg.orientation.y + msg.orientation.z*msg.orientation.z));
  double pitch = asin(2.0*(msg.orientation.w*msg.orientation.y - msg.orientation.z*msg.orientation.x));
  double roll = atan2(2.0*(msg.orientation.w*msg.orientation.x + msg.orientation.y*msg.orientation.z), 1.0 - 2.0*(msg.orientation.x*msg.orientation.x + msg.orientation.y*msg.orientation.y));

  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Roll: ");
  Serial.println(roll);

  stepper.moveTo((yaw * (180 / PI) * SPR) / 360); //Set the target motor position
  stepper.runToPosition(); // Run the motor to the target position
 /*   while (stepper.distanceToGo() != 0) {
    stepper.run();
  }*/
}

ros::Subscriber<geometry_msgs::Pose> sub("QUAT", messageCb);

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  stepper.setMaxSpeed(1000);//Set the maximum motor speed in steps per second
  stepper.setAcceleration(200);//Set the maximum acceleration in steps per second^2
}
 
void loop() {
  nh.spinOnce();
  delay(1);
/*  stepper.moveTo(3*SPR); //Set the target motor position (i.e. turn motor for 3 full revolutions)
  stepper.runToPosition(); // Run the motor to the target position 
  delay(1000);
  stepper.moveTo(-3*SPR);//Same as above: Set the target motor position (i.e. turn motor for 3 full revolutions)
  stepper.runToPosition(); // Run the motor to the target position */
 // delay(1000);
}
