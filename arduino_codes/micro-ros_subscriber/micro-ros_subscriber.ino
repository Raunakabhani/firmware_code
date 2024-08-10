//1.include required library for working of electronics of robot
#include <PID_v1.h>  // Library: PID by Brett Beauregard
#include <micro_ros_arduino.h> // Include the Micro-ROS for Arduino library

//2.include required libraries for microros
#include <stdio.h> // Include standard input-output library
#include <rcl/rcl.h> // Include ROS Client Library (RCL)
#include <rcl/error_handling.h> // Include RCL error handling functionalities
#include <rclc/rclc.h> // Include ROS Client Library for C (RCLC)
#include <rclc/executor.h> // Include ROS Client Library Executor functionalities

//3.include required message type
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/msg/int32.h> // Include Int32 message type from std_msgs package



//4. doubt
// Declare ROS objects and variables
rcl_subscription_t subscriber; // ROS subscription handle
rcl_publisher_t publisher;     //ROS publisher handle
rclc_executor_t executor; // ROS executor object
rclc_support_t support; // ROS support object
rcl_allocator_t allocator; // ROS allocator object
rcl_node_t node; // ROS node object
rcl_timer_t timer; // ROS timer object


std_msgs__msg__Int32 msg; // Int32 message object


//5.Pin definition
#define LED_PIN 12 // Define LED pin number

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}} // Macro for checking return codes from ROS functions
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Function to handle errors
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle the LED pin state
    delay(100); // Wait for 100 milliseconds
  }
}

// Callback function for subscription
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin; // Cast the received message to Int32 type
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH); // Set the LED state based on the received data value  
}

// Arduino setup function
void setup() {
  set_microros_transports(); // Set up Micro-ROS transports
  
  pinMode(LED_PIN, OUTPUT); // Set LED pin as output
  digitalWrite(LED_PIN, HIGH); // Turn off LED initially
  
  delay(2000); // Wait for 2 seconds

  allocator = rcl_get_default_allocator(); // Get default ROS allocator

  // Initialize support object
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize ROS node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Initialize ROS subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_subscriber"));

  // Initialize ROS executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

// Arduino main loop
void loop() {
  delay(100); // Wait for 100 milliseconds
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Spin the ROS executor
}
