#include <Servo.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle node_handle;

Servo robot_servos[5];

int servo_pins[5] = {10, 9, 6, 5, 3}; // PWM Pins on Arduino Uno
int mid_positions[5] = {100, 100, 100, 90, 100};
int SERVO_CURRENT_POSITIONS[5];

float TARGET_JOINT_POSITIONS[5] = {0,0,0,0,0};

// Convert the joint state values to degrees, adjust for the center and write to the servo
void writeServos() {
  for (int j = 0; j < 5; j++) {
    int target_angle;
    if (j == 2) {
      // Due to difference in mounting directions
      target_angle = - TARGET_JOINT_POSITIONS[j]*(180/3.14) + mid_positions[j];
    } else {
      target_angle = TARGET_JOINT_POSITIONS[j]*(180/3.14) + mid_positions[j];
    }
    robot_servos[j].write(target_angle);
    SERVO_CURRENT_POSITIONS[j] = target_angle;
  }
  node_handle.spinOnce();
}

// Subscriber Callback to store the jointstate position values in the global variables
void servoControlSubscriberCallbackJointState(const sensor_msgs::JointState& msg) {
  TARGET_JOINT_POSITIONS[0] = msg.position[0];
  TARGET_JOINT_POSITIONS[1] = msg.position[1];
  TARGET_JOINT_POSITIONS[2] = msg.position[2];
  TARGET_JOINT_POSITIONS[3] = msg.position[3];
  TARGET_JOINT_POSITIONS[4] = msg.position[4];
  // Call the method to write the joint positions to the servo motors
  writeServos();

}


ros::Subscriber<sensor_msgs::JointState> servo_control_subscriber_joint_state("joint_states", &servoControlSubscriberCallbackJointState);

void setup() {
  // Initial the servo motor connections and initialize them at home position
  for (unsigned int i = 0; i < 5; i++) {
    robot_servos[i].attach(servo_pins[i]);
    robot_servos[i].write(mid_positions[i]);
    SERVO_CURRENT_POSITIONS[i] = mid_positions[i];
  }

  // Set the communication BaudRate and start the node
  node_handle.getHardware()->setBaud(115200);
  node_handle.initNode();
  node_handle.subscribe(servo_control_subscriber_joint_state);
}

void loop() {
  // Keep calling the spinOnce() method in this infinite loop to stay tightly coupled with the ROS Serial
  node_handle.spinOnce();
  delay(1);
}
