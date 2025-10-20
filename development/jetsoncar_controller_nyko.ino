/*
 * rosserial Controlling a servo motor and light intensity with Joystick node
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <Servo.h>

Servo steering_servo, throttle_servo;

ros::NodeHandle nh;
const int minSteering = 30;
const int maxSteering = 320;  //150;
const int minThrottle = 0;
const int maxThrottle = 320;  //150;
int throttleVal = 0;
int steeringVal = 0;

std_msgs::Int32 ctrl_instruction;
ros::Publisher chatter("chatter", &ctrl_instruction);

//Arduino 'map' function for floating point
double fmap(double toMap, double input_min, double input_max, double output_min, double output_max) {
  return (toMap - input_min) * (output_max - output_min) / (input_max - input_min) + output_min;
}


void car_controller(const geometry_msgs::Twist& twist_msg) {
  if (twist_msg.angular.z >= 0) {
    steeringVal = (int)fmap(twist_msg.angular.z, 0, 100.0, 97, minSteering);
  } else {
    //steeringVal = (int)fmap(twist_msg.angular.z, 10.0, -10.0, minSteering, maxSteering);
    steeringVal = (int)fmap(twist_msg.angular.z, -100, 0, maxSteering, 97);
  }
  ctrl_instruction.data = steeringVal;
  chatter.publish(&ctrl_instruction);
  if (steeringVal < minSteering) {

    steeringVal = minSteering;
  }

  if (steeringVal > maxSteering) {

    steeringVal = maxSteering;
  }

  steering_servo.write(steeringVal);

  if (twist_msg.linear.x >= 0) {

    throttleVal = (int)fmap(twist_msg.linear.x, 0, 10, 90, maxThrottle);

  }

  else {
    throttleVal = (int)fmap(twist_msg.linear.x, 0, -10, 90, minThrottle);
  }
  ctrl_instruction.data = throttleVal;
  chatter.publish(&ctrl_instruction);
  if (throttleVal < minThrottle) {

    throttleVal = minThrottle;
  }

  if (throttleVal > maxThrottle) {

    throttleVal = maxThrottle;
  }
  throttle_servo.write(throttleVal);
}



ros::Subscriber<geometry_msgs::Twist> driveSubscriber("jetsoncar_instructions", &car_controller);

void setup() {

  Serial.begin(57600);
  steering_servo.attach(3);
  throttle_servo.attach(11);
  steering_servo.write(97);
  throttle_servo.write(90);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(driveSubscriber);
  delay(1000);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
