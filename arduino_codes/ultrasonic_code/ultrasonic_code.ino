//################### 1.DNC include required library for working of electronics of robot ####################################################################################################################
#include <micro_ros_arduino.h> // Include the Micro-ROS for Arduino library

//################### 2.DNC include required libraries for microros #########################################################################################################################################
#include <stdio.h> // Include standard input-output library
#include <rcl/rcl.h> // Include ROS Client Library (RCL)
#include <rcl/error_handling.h> // Include RCL error handling functionalities
#include <rclc/rclc.h> // Include ROS Client Library for C (RCLC)
#include <rclc/executor.h> // Include ROS Client Library Executor functionalities

//#################### 3.DNC include required message type (name of library is case sensitive and location of all msg type is inside micro ros lib,you can search message type to get correct name and path.)
#include <std_msgs/msg/float64.h>

//#################### 4.Create message object (All msg object name should be different,no matter if all 4 vel_msg is using same msg type , their object name should be different)###########################
std_msgs__msg__Float64 middle_ultrasonic_msg;

//################### 5. Declare ROS objects and variables (All object name of publisher and subscriber should be different ###############################################################################
rcl_publisher_t middle_ultrasonic_publisher;     //ROS publisher handle (for )

// what is this ????
rclc_executor_t executor; // ROS executor object (responsible for managing the execution of tasks, such as calling callback functions when messages are received or timers expire)
rclc_support_t support; // ROS support object (provides support for creating nodes, setting up communication, and more.)
rcl_allocator_t allocator; // ROS allocator object  (used to manage memory allocation and deallocation in a deterministic manner)
rcl_node_t node; // ROS node object
rcl_timer_t timer; // ROS timer object

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;

// defines variables
int distance_feedback_period = 50; //velocity is updated every 50 milli seconds 
unsigned long time_now = 0;
long duration;
float middle_ultrasonic_distance;

//9. Doubt  Macros to handle micro-ROS return codes, error handling should be customized for the target syst 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}} 
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
void error_loop(){
//  while(1){
//    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle the LED pin state
//    delay(100); // Wait for 100 milliseconds
//  }
}

//########### 15.Timer callback for publisher ##################################################################################################################################################################
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    middle_ultrasonic_msg.data = middle_ultrasonic_distance;
//    rf_vel_msg.data = rf_velocity;
//    lb_vel_msg.data = lb_velocity;


    RCSOFTCHECK(rcl_publish(&middle_ultrasonic_publisher, &middle_ultrasonic_msg, NULL));    
      middle_ultrasonic_msg.data;
//    RCSOFTCHECK(rcl_publish(&rf_vel_publisher, &rf_vel_msg, NULL));    
//      rf_vel_msg.data;
//    RCSOFTCHECK(rcl_publish(&lb_vel_publisher, &lb_vel_msg, NULL));    
//      lb_vel_msg.data;

  }
}

void setup() {
  // Initialize serial for debugging
  
  set_microros_transports(); // Only required when sending data to esp over wifi
  allocator = rcl_get_default_allocator(); // Get default ROS allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); // Initialize support object
  RCCHECK(rclc_node_init_default(&node, "ultrasonic_arduino_node", "", &support));// Create ROS node
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &middle_ultrasonic_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "middle_ultrasonic_distance"));
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(57600); // Starts the serial communication

   // create timer,
   const unsigned int timer_timeout = 100;
   RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

}


void distance_measurement() {


  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  middle_ultrasonic_distance= duration*0.034/2;
  
  //publishing data
 
  delay(100);


  if (millis() >= time_now + distance_feedback_period) {
    time_now += distance_feedback_period;

    middle_ultrasonic_msg.data= middle_ultrasonic_distance;
//    rf_vel_msg.data= rf_velocity;
//    lb_vel_msg.data= lb_velocity;
  }
  
}

void loop() {

  distance_measurement(),
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Spin the ROS executor
  
}
