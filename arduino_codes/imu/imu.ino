//################### 1.DNC include required library for working of electronics of robot ####################################################################################################################
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <micro_ros_arduino.h> // Include the Micro-ROS for Arduino library

//################### 2.DNC include required libraries for microros #########################################################################################################################################
#include <stdio.h> // Include standard input-output library
#include <rcl/rcl.h> // Include ROS Client Library (RCL)
#include <rcl/error_handling.h> // Include RCL error handling functionalities
#include <rclc/rclc.h> // Include ROS Client Library for C (RCLC)
#include <rclc/executor.h> // Include ROS Client Library Executor functionalities

//#################### 3.DNC include required message type (name of library is case sensitive and location of all msg type is inside micro ros lib,you can search message type to get correct name and path.)
#include <sensor_msgs/msg/imu.h>

//#################### 4.Create message object (All msg object name should be different,no matter if all 4 vel_msg is using same msg type , their object name should be different)###########################
sensor_msgs__msg__Imu imu_msg;

//################### 5. Declare ROS objects and variables (All object name of publisher and subscriber should be different ###############################################################################
rcl_publisher_t imu_publisher;

rclc_executor_t executor; // ROS executor object (responsible for managing the execution of tasks, such as calling callback functions when messages are received or timers expire)
rclc_support_t support; // ROS support object (provides support for creating nodes, setting up communication, and more.)
rcl_allocator_t allocator; // ROS allocator object  (used to manage memory allocation and deallocation in a deterministic manner)
rcl_node_t node; // ROS node object
rcl_timer_t timer; // ROS timer object

//9. Doubt  Macros to handle micro-ROS return codes, error handling should be customized for the target syst 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}} 
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
void error_loop(){
//  while(1){
//    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle the LED pin state
//    delay(100); // Wait for 100 milliseconds
//  }
}

// Define the BNO055 I2C address and create an instance of the sensor
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void)last_call_time;

  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  sensors_event_t orientationData, angVelocityData, linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  imu_msg.header.frame_id.data = (char *)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
  imu_msg.header.stamp.sec = 0;  // Fill with appropriate timestamp
  imu_msg.header.stamp.nanosec = 0;

  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();
  
  imu_msg.angular_velocity.x = angVelocityData.gyro.x;
  imu_msg.angular_velocity.y = angVelocityData.gyro.y;
  imu_msg.angular_velocity.z = angVelocityData.gyro.z;

  imu_msg.linear_acceleration.x = linearAccelData.acceleration.x;
  imu_msg.linear_acceleration.y = linearAccelData.acceleration.y;
  imu_msg.linear_acceleration.z = linearAccelData.acceleration.z;

  rcl_publish(&imu_publisher, &imu_msg, NULL);
}


void setup() {

    //DNC
  Serial.begin(115200);
  set_microros_transports(); // Only required when sending data to esp over wifi
  allocator = rcl_get_default_allocator(); // Get default ROS allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); // Initialize support object
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));// Create ROS node
  

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data"));

  // Initialize the sensor
  // if (!bno.begin()) {
  //   Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //   while (1);
  // }
  delay(1000);
  bno.setExtCrystalUse(true);

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialize executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  // Spin the executor to handle callbacks
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(BNO055_SAMPLERATE_DELAY_MS))); // Spin the ROS executor
}


