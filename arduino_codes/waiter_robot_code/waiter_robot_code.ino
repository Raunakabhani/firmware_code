//################### 1.DNC include required library for working of electronics of robot ####################################################################################################################
#include <PID_v1.h>  // Library: PID by Brett Beauregard
#include <micro_ros_arduino.h> // Include the Micro-ROS for Arduino library

//################### 2.DNC include required libraries for microros #########################################################################################################################################
#include <stdio.h> // Include standard input-output library
#include <rcl/rcl.h> // Include ROS Client Library (RCL)
#include <rcl/error_handling.h> // Include RCL error handling functionalities
#include <rclc/rclc.h> // Include ROS Client Library for C (RCLC)
#include <rclc/executor.h> // Include ROS Client Library Executor functionalities

//#################### 3.DNC include required message type (name of library is case sensitive and location of all msg type is inside micro ros lib,you can search message type to get correct name and path.)
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

//#################### 4.Create message object (All msg object name should be different,no matter if all 4 vel_msg is using same msg type , their object name should be different)###########################
std_msgs__msg__Bool pid_bool_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32 lf_vel_msg;
std_msgs__msg__Float32 rf_vel_msg;
std_msgs__msg__Float32 lb_vel_msg;
std_msgs__msg__Float32 rb_vel_msg;
std_msgs__msg__String string_msg;
std_msgs__msg__Int32 int32_msg;

//################### 5. Declare ROS objects and variables (All object name of publisher and subscriber should be different ###############################################################################
rcl_subscription_t pid_control_subscriber; // ROS subscription handle (for pid control)
rcl_subscription_t cmd_vel_subscriber;     // ROS subscription handle (for cmd vel)
rcl_publisher_t lf_vel_publisher;     //ROS publisher handle (for lf_vel from encoder)
rcl_publisher_t rf_vel_publisher;     //ROS publisher handle (for rf_vel from encoder)
rcl_publisher_t lb_vel_publisher;     //ROS publisher handle (for lb vel from encoder)
rcl_publisher_t rb_vel_publisher;     //ROS publisher handle (for rb vel from encoder)

// what is this ????
rclc_executor_t executor; // ROS executor object (responsible for managing the execution of tasks, such as calling callback functions when messages are received or timers expire)
rclc_support_t support; // ROS support object (provides support for creating nodes, setting up communication, and more.)
rcl_allocator_t allocator; // ROS allocator object  (used to manage memory allocation and deallocation in a deterministic manner)
rcl_node_t node; // ROS node object
rcl_timer_t timer; // ROS timer object

//################### 6.To remove PWM noice ##############################################################################################################################################################
bool PID_control = true; //Flag to enable/disable PID control

//what is this?????
const int PWM_frequency = 15000;
const int pwm_resolution = 8;

const int lf_pwm_channel = 3;
const int lb_pwm_channel = 4;
const int rf_pwm_channel = 5;
const int rb_pwm_channel = 6;


//################## 7.Pin definition ######################################################################################################################################################################
#define LED_PIN 12 // Define LED pin number

#define left_front_dir 26
#define left_front_pwm 25
#define left_back_dir 13
#define left_back_pwm 32

#define right_front_dir 19
#define right_front_pwm 4
#define right_back_dir 18
#define right_back_pwm 23

#define left_front_enA 34
#define left_front_enB 36
#define left_back_enA 35
#define left_back_enB 39

#define right_front_enA 14
#define right_front_enB 27
#define right_back_enA 16
#define right_back_enB 17

//################ 8.Wheel,Motor,Encoder parameter declaration #############################################################################################################################################
float lf_PPR = 498.4;  // Left encoders Pulse Per Revolution  (depends on selected encoder)
float rf_PPR = 498.4;  // Right encoders Pulse Per Revolution  (depends on selected encoder)
float lb_PPR = 498.4;  // Left encoders Pulse Per Revolution  (depends on selected encoder)
float rb_PPR = 498.4;  // Right lf_PPR
const int motor_rpm = 100; //rated speed of motor (depends on selected encoder) 
const float wheel_dia = 0.105; //wheel diameter in meter
const float wheel_separation_width = 0.475; //wheel separation width in meter
const float wheel_separation_length = 0.555; //wheel separation length in meter
const int max_pwm = 255; //max value of pwm
const int min_pwm = 0;   //min value of pwm
float min_vel = 0.0;          // Unit m/s
float max_vel = 0.549;        // Unit m/s
float max_vel_limit = 0.549;  //Set limit to control Speed in m/s
float max_vel_int = max_vel * 1000;
const float wheel_radius = wheel_dia / 2;
const float circumference_of_wheel = 2 * 3.145 * wheel_radius;
const float max_speed = (circumference_of_wheel * motor_rpm) / 60;


//what is this ??
bool rf_state = 0, lf_state = 0, rb_state = 0, lb_state = 0;

//################ 9.PID Parameters ######################################################################################################################################################################
// setpoint =  desired velocities for respective wheel
// PID_input = actual velocities measured by the encoder
// PWM_ouput = pwm values outputted by the PID controller to achieve the desired velocities
double rf_Setpoint, rf_tick_PID_input, rf_motor_pwm_output;
double lf_Setpoint, lf_tick_PID_input, lf_motor_pwm_output;
double rb_Setpoint, rb_tick_PID_input, rb_motor_pwm_output;
double lb_Setpoint, lb_tick_PID_input, lb_motor_pwm_output;

//PID constants
double Kp = 0.1, Ki = 4, Kd = 0;

//passing necessary parameters in PID lib function
PID rf_PID(&rf_tick_PID_input, &rf_motor_pwm_output, &rf_Setpoint, Kp, Ki, Kd, DIRECT);
PID lf_PID(&lf_tick_PID_input, &lf_motor_pwm_output, &lf_Setpoint, Kp, Ki, Kd, DIRECT);
PID rb_PID(&rb_tick_PID_input, &rb_motor_pwm_output, &rb_Setpoint, Kp, Ki, Kd, DIRECT);
PID lb_PID(&lb_tick_PID_input, &lb_motor_pwm_output, &lb_Setpoint, Kp, Ki, Kd, DIRECT);


//9. Doubt  Macros to handle micro-ROS return codes, error handling should be customized for the target syst 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}} 
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
void error_loop(){
//  while(1){
//    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle the LED pin state
//    delay(100); // Wait for 100 milliseconds
//  }
}

//############## 12.Control and Encoder Defination #########################################################################################################################################################

//from encoder_ticks.ion we get 2 parameters encoder_ticks_count and encoder_ticks_pub
volatile int rf_ticks_count, lf_ticks_count, rb_ticks_count, lb_ticks_count;
volatile int rf_ticks_pub, lf_ticks_pub, rb_ticks_pub, lb_ticks_pub;

//velocity extracted from cmd_vel (which is a subscriber)
float linear_vel, angular_vel, side_vel;

// declaring direction of wheel which will be checked in encoder_ticks.ino (True = Forward; False = Reverse)
boolean Direction_lf = true, Direction_rf = true, Direction_lb = true, Direction_rb = true;

// declaring previous direction of wheel  (True = Forward; False = Reverse)
int prev_dir_rf = false, prev_dir_lf = false, prev_dir_rb = false, prev_dir_lb = false;

// Minumum and maximum values for 16-bit integers (so when min and max value is achieved in encoder, it starts counting from starting 
const int encoder_minimum = -32768, encoder_maximum = 32767;

float lf_RPM = 0, lf_velocity = 0;
int lf_velocity_int = 0;

float rf_RPM = 0, rf_velocity = 0;
int rf_velocity_int = 0;

float lb_RPM = 0, lb_velocity = 0;
int lb_velocity_int = 0;

float rb_RPM = 0, rb_velocity = 0;
int rb_velocity_int = 0;

unsigned long previousMillis = 0;
const long period = 100;

//############# 13.Callback function for pid control subscription #############################################################################################################################################
void pidControlCallback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  PID_control = msg->data;
}

//############ 14.Callback function for cmd_vel subscription ##################################################################################################################################################
void onTwist(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    linear_vel = msg->linear.x;
    angular_vel = msg->angular.z;
    side_vel = msg->linear.y;
}

//########### 15.Timer callback for publisher ##################################################################################################################################################################
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    lf_vel_msg.data = lf_velocity;
    rf_vel_msg.data = rf_velocity;
    lb_vel_msg.data = lb_velocity;
    rb_vel_msg.data = rb_velocity;
    RCSOFTCHECK(rcl_publish(&lf_vel_publisher, &lf_vel_msg, NULL));    
      lf_vel_msg.data;
    RCSOFTCHECK(rcl_publish(&rf_vel_publisher, &rf_vel_msg, NULL));    
      rf_vel_msg.data;
    RCSOFTCHECK(rcl_publish(&lb_vel_publisher, &lb_vel_msg, NULL));    
      lb_vel_msg.data;
    RCSOFTCHECK(rcl_publish(&rb_vel_publisher, &rb_vel_msg, NULL));    
      rb_vel_msg.data;
  }
}

//######## 16.RPM Velocity calculation ###########################################################################################################################################################################
int vel_feedback_period = 50; //velocity is updated every 50 milli seconds 
unsigned long time_now = 0;

void RPM_velocity_calc() {
  // millis return the number of milliseconds at the time, the esp board begins running the current program.
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= period) {
    previousMillis = currentMillis;

    lf_RPM = ((lf_ticks_count) / lf_PPR) * (60 * (1000.0 / period));
    lf_velocity = wheel_radius * lf_RPM * 0.10472;

    rf_RPM = ((rf_ticks_count) / rf_PPR) * (60 * (1000.0 / period));
    rf_velocity = wheel_radius * rf_RPM * 0.10472;

    lb_RPM = ((lb_ticks_count) / lb_PPR) * (60 * (1000.0 / period));
    lb_velocity = wheel_radius * lb_RPM * 0.10472;

    rb_RPM = ((rb_ticks_count) / rb_PPR) * (60 * (1000.0 / period));
    rb_velocity = wheel_radius * rb_RPM * 0.10472;

    lf_ticks_count = 0;
    rf_ticks_count = 0;
    lb_ticks_count = 0;
    rb_ticks_count = 0;
  }
}


//12.Arduino setup function
void setup() {
  //DNC
  set_microros_transports(); // Only required when sending data to esp over wifi
  allocator = rcl_get_default_allocator(); // Get default ROS allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); // Initialize support object
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));// Create ROS node
  

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &lf_vel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "lf_vel"));

   RCCHECK(rclc_publisher_init_default(
    &rf_vel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rf_vel"));

   RCCHECK(rclc_publisher_init_default(
    &lb_vel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "lb_vel"));

   RCCHECK(rclc_publisher_init_default(
    &rb_vel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rb_vel"));

   // Create ROS subscriber for PID controller
   RCCHECK(rclc_subscription_init_default(
    &pid_control_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "pid_control"));
    
   // Create ROS subscriber for cmd vel
   RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

    // create timer,
   const unsigned int timer_timeout = 100;
   RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialize ROS executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &pid_control_subscriber, &pid_bool_msg, &pidControlCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &onTwist, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
  pinMode(right_front_enA, INPUT);
  pinMode(right_front_enB, INPUT);

  pinMode(right_back_enA, INPUT);
  pinMode(right_back_enB, INPUT);

  pinMode(left_front_enA, INPUT);
  pinMode(left_front_enB, INPUT);

  pinMode(left_back_enA, INPUT);
  pinMode(left_back_enB, INPUT);
//
  attachInterrupt(digitalPinToInterrupt(right_front_enA), rf_ticks, RISING);
  attachInterrupt(digitalPinToInterrupt(right_back_enA), rb_ticks, RISING);
  attachInterrupt(digitalPinToInterrupt(left_front_enA), lf_ticks, RISING);
  attachInterrupt(digitalPinToInterrupt(left_back_enA), lb_ticks, RISING);

  pinMode(right_front_pwm, OUTPUT);
  pinMode(right_front_dir, OUTPUT);
  pinMode(right_back_pwm, OUTPUT);
  pinMode(right_back_dir, OUTPUT);

  pinMode(left_front_pwm, OUTPUT);
  pinMode(left_front_dir, OUTPUT);
  pinMode(left_back_pwm, OUTPUT);
  pinMode(left_back_dir, OUTPUT);

  ledcSetup(lf_pwm_channel, PWM_frequency, pwm_resolution);
  ledcSetup(lb_pwm_channel, PWM_frequency, pwm_resolution);
  ledcSetup(rf_pwm_channel, PWM_frequency, pwm_resolution);
  ledcSetup(rb_pwm_channel, PWM_frequency, pwm_resolution);

  ledcAttachPin(left_front_pwm, lf_pwm_channel);
  ledcAttachPin(left_back_pwm, lb_pwm_channel);
  ledcAttachPin(right_front_pwm, rf_pwm_channel);
  ledcAttachPin(right_back_pwm, rb_pwm_channel);


  ledcWrite(lf_pwm_channel, 0);
  ledcWrite(rf_pwm_channel, 0);
  ledcWrite(lb_pwm_channel, 0);
  ledcWrite(rb_pwm_channel, 0);

  rf_PID.SetMode(AUTOMATIC);
  rf_PID.SetTunings(Kp, Ki, Kd);

  lf_PID.SetMode(AUTOMATIC);
  lf_PID.SetTunings(Kp, Ki, Kd);

  rb_PID.SetMode(AUTOMATIC);
  rb_PID.SetTunings(Kp, Ki, Kd);

  lb_PID.SetMode(AUTOMATIC);
  lb_PID.SetTunings(Kp, Ki, Kd);


}

//######## 17.PID execution ######################################################################################################################################################################################
void PID_Execution() {

  float lf_vel = (linear_vel - side_vel - ((wheel_separation_width + wheel_separation_length) * angular_vel / 2));
  float rf_vel = (linear_vel + side_vel + ((wheel_separation_width + wheel_separation_length) * angular_vel / 2));
  float lb_vel = (linear_vel + side_vel - ((wheel_separation_width + wheel_separation_length) * angular_vel / 2));
  float rb_vel = (linear_vel - side_vel + ((wheel_separation_width + wheel_separation_length) * angular_vel / 2));

  int lf_PWM = _max(_min(((abs(lf_vel) / max_speed) * max_pwm), max_pwm), min_pwm);
  int rf_PWM = _max(_min(((abs(rf_vel) / max_speed) * max_pwm), max_pwm), min_pwm);
  int lb_PWM = _max(_min(((abs(lb_vel) / max_speed) * max_pwm), max_pwm), min_pwm);
  int rb_PWM = _max(_min(((abs(rb_vel) / max_speed) * max_pwm), max_pwm), min_pwm);

  rf_velocity_int = abs(rf_velocity * 1000);
  lf_velocity_int = abs(lf_velocity * 1000);
  rb_velocity_int = abs(rb_velocity * 1000);
  lb_velocity_int = abs(lb_velocity * 1000);

  if (lf_vel == 0) {
    digitalWrite(left_front_dir, prev_dir_lf);
  } else if (lf_vel >= 0) {
    digitalWrite(left_front_dir, 0);
    prev_dir_lf = 0;
  } else {
    digitalWrite(left_front_dir, 1);
    prev_dir_lf = 1;
  }

  if (rf_vel == 0) {
    digitalWrite(right_front_dir, prev_dir_rf);
  } else if (rf_vel >= 0) {
    digitalWrite(right_front_dir, 1);
    prev_dir_rf = 1;
  } else {
    digitalWrite(right_front_dir, 0);
    prev_dir_rf = 0;
  }

  if (lb_vel == 0) {
    digitalWrite(left_back_dir, prev_dir_lb);
  } else if (lb_vel >= 0) {
    digitalWrite(left_back_dir, 0);
    prev_dir_lb = 0;
  } else {
    digitalWrite(left_back_dir, 1);
    prev_dir_lb = 1;
  }

  if (rb_vel == 0) {
    digitalWrite(right_back_dir, prev_dir_rb);
  } else if (rb_vel >= 0) {
    digitalWrite(right_back_dir, 1);
    prev_dir_rb = 1;
  } else {
    digitalWrite(right_back_dir, 0);
    prev_dir_rb = 0;
  }

  if (PID_control == true) {

    rf_tick_PID_input = map(rf_velocity_int, min_vel, max_vel_int, min_pwm, max_pwm);
    lf_tick_PID_input = map(lf_velocity_int, min_vel, max_vel_int, min_pwm, max_pwm);
    rb_tick_PID_input = map(rb_velocity_int, min_vel, max_vel_int, min_pwm, max_pwm);
    lb_tick_PID_input = map(lb_velocity_int, min_vel, max_vel_int, min_pwm, max_pwm);

    if (max_vel_limit <= abs(lf_vel)) {
      lf_Setpoint = (max_vel_limit * 1000) / (max_vel_int / max_pwm);
    } else {
      lf_Setpoint = (abs(lf_vel) * 1000) / (max_vel_int / max_pwm);
    }
    if (max_vel_limit <= abs(rb_vel)) {
      rb_Setpoint = (max_vel_limit * 1000) / (max_vel_int / max_pwm);
    } else {
      rb_Setpoint = (abs(rb_vel) * 1000) / (max_vel_int / max_pwm);
    }
    if (max_vel_limit <= abs(lb_vel)) {
      lb_Setpoint = (max_vel_limit * 1000) / (max_vel_int / max_pwm);
    } else {
      lb_Setpoint = (abs(lb_vel) * 1000) / (max_vel_int / max_pwm);
    }
    if (max_vel_limit <= abs(rf_vel)) {
      rf_Setpoint = (max_vel_limit * 1000) / (max_vel_int / max_pwm);
    } else {
      rf_Setpoint = (abs(rf_vel) * 1000) / (max_vel_int / max_pwm);
    }

    rf_PID.Compute();
    lf_PID.Compute();
    rb_PID.Compute();
    lb_PID.Compute();

    ledcWrite(lf_pwm_channel, lf_motor_pwm_output);
    ledcWrite(rf_pwm_channel, rf_motor_pwm_output);
    ledcWrite(lb_pwm_channel, lb_motor_pwm_output);
    ledcWrite(rb_pwm_channel, rb_motor_pwm_output);
  } else {
    ledcWrite(lf_pwm_channel, lf_PWM);
    ledcWrite(rf_pwm_channel, rf_PWM);
    ledcWrite(lb_pwm_channel, lb_PWM);
    ledcWrite(rb_pwm_channel, rb_PWM);
  }

  if (millis() >= time_now + vel_feedback_period) {
    time_now += vel_feedback_period;

    lf_vel_msg.data= lf_velocity;
    rf_vel_msg.data= rf_velocity;
    lb_vel_msg.data= lb_velocity;
    rb_vel_msg.data= rb_velocity;

//    lf_vel_publisher.publish(&lf_vel_msg);
//    rf_vel_publisher.publish(&rf_vel_msg);
//    lb_vel_publisher.publish(&lb_vel_msg);
//    rb_vel_publisher.publish(&rb_vel_msg);
  }
}

// Arduino main loop
void loop() {
  RPM_velocity_calc();
  delay(100); // Wait for 100 milliseconds
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Spin the ROS executor
  PID_Execution();
}
