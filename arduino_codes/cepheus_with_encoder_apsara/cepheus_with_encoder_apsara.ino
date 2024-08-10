//**********necessary library**************************
#include <PID_v1.h>  // Library: PID by Brett Beauregard
#include <ros.h>

//**********necessary message type*********************
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

//*********** Flag to enable/disable PID control****************
bool PID_control = true;

// ********* To remove low PWM noise ******************
const int PWM_frequency = 15000;  //frequency at which pwm signals are generated 
const int pwm_resolution = 8;

const int lf_pwm_channel = 3;
const int lb_pwm_channel = 4;
const int rf_pwm_channel = 5;
const int rb_pwm_channel = 6;

//********** defining motor pins **********************
#define right_front_dir 26
#define right_front_pwm 25
#define right_back_dir 13
#define right_back_pwm 32

#define left_front_dir 19
#define left_front_pwm 4
#define left_back_dir 18
#define left_back_pwm 23

//******** defining encoder pin ***********************
#define left_front_enA 14
#define left_front_enB 27
#define left_back_enA 16
#define left_back_enB 17

#define right_front_enA 34
#define right_front_enB 36
#define right_back_enA 35
#define right_back_enB 39

// ************ Differential parameters ****************
// ***************  VERY IMPORTANT  ********************

float lf_PPR = 498.4;  // Left Encoders Pulse Per Revolution  (depends on selected encoder)
float rf_PPR = 498.4;  // Right Encoders Pulse Per Revolution (depends on selected encoder)
float lb_PPR = 498.4;  // Left Encoders Pulse Per Revolution  (depends on selected encoder)
float rb_PPR = 498.4;  // Right Encoders Pulse Per Revolution (depends on selected encoder)  
const int motor_rpm = 100; //rated speed of motor (depends on selected encoder) 
const float wheel_dia = 0.105; //wheel diameter in meter
const float wheel_separation_width = 0.475; //wheel separation width in meter
const float wheel_separation_length = 0.555; //wheel separation length in meter
const int max_pwm = 255;  //max value of pwm 
const int min_pwm = 0;    //min value of pwm
float min_vel = 0.0;          // Unit m/s
float max_vel = 0.549;        // Unit m/s
float max_vel_limit = 0.549;  //Set limit to control Speed in m/s
float max_vel_int = max_vel * 1000;
const float wheel_radius = wheel_dia / 2;
const float circumference_of_wheel = 2 * 3.145 * wheel_radius;
const float max_speed = (circumference_of_wheel * motor_rpm) / 60;

bool rf_state = 0, lf_state = 0, rb_state = 0, lb_state = 0;

// *************** PID Parameters **********************

// setpoint =  desired velocities for respective wheel
// PID_input = actual velocities measured by the encoder
// PWM_ouput = pwm values outputted by the PID controller to achieve the desired velocities

double rf_Setpoint, rf_tick_PID_input, rf_motor_pwm_output;
double lf_Setpoint, lf_tick_PID_input, lf_motor_pwm_output;
double rb_Setpoint, rb_tick_PID_input, rb_motor_pwm_output;
double lb_Setpoint, lb_tick_PID_input, lb_motor_pwm_output;

double Kp = 0.1, Ki = 4, Kd = 0;

//pid controller instance for each wheel
PID rf_PID(&rf_tick_PID_input, &rf_motor_pwm_output, &rf_Setpoint, Kp, Ki, Kd, DIRECT);
PID lf_PID(&lf_tick_PID_input, &lf_motor_pwm_output, &lf_Setpoint, Kp, Ki, Kd, DIRECT);
PID rb_PID(&rb_tick_PID_input, &rb_motor_pwm_output, &rb_Setpoint, Kp, Ki, Kd, DIRECT);
PID lb_PID(&lb_tick_PID_input, &lb_motor_pwm_output, &lb_Setpoint, Kp, Ki, Kd, DIRECT);


// *************** Vel Publishers **********************

std_msgs::Float32 lf_vel_msg;
ros::Publisher lf_vel_pub("lf_vel", &lf_vel_msg);

std_msgs::Float32 rf_vel_msg;
ros::Publisher rf_vel_pub("rf_vel", &rf_vel_msg);

std_msgs::Float32 lb_vel_msg;
ros::Publisher lb_vel_pub("lb_vel", &lb_vel_msg);

std_msgs::Float32 rb_vel_msg;
ros::Publisher rb_vel_pub("rb_vel", &rb_vel_msg);

ros::NodeHandle nh;

// ****** Control and Encoder Definations **************

//ticks_count = encoder tick counts for each wheel
volatile int rf_ticks_count, lf_ticks_count, rb_ticks_count, lb_ticks_count;

//published encoder ticks for each wheel 
volatile int rf_ticks_pub, lf_ticks_pub, rb_ticks_pub, lb_ticks_pub;

//velocities from ros command 
float linear_vel, angular_vel, side_vel;

//direction flag for each wheel 
// True = Forward; False = Reverse
boolean Direction_lf = true, Direction_rf = true, Direction_lb = true, Direction_rb = true;

//previous direction state for each wheel 
int prev_dir_rf = false, prev_dir_lf = false, prev_dir_rb = false, prev_dir_lb = false;

// Minumum and maximum values for 16-bit integers
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

//******************************************************
void pidControlCallback(const std_msgs::Bool& msg) {
  PID_control = msg.data;
}
// Create a subscriber for the "PID_control" topic
ros::Subscriber<std_msgs::Bool> pidControlSubscriber("pid_control", &pidControlCallback);

void onTwist(const geometry_msgs::Twist& msg) {
  linear_vel = msg.linear.x;
  angular_vel = msg.angular.z;
  side_vel = msg.linear.y;
}
ros::Subscriber<geometry_msgs::Twist> sub_cmd("cmd_vel", onTwist);

int vel_feedback_period = 50;
unsigned long time_now = 0;

void RPM_velocity_calc() {

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

void setup() {

  pinMode(right_front_enA, INPUT_PULLUP);
  pinMode(right_front_enB, INPUT_PULLUP);

  pinMode(right_back_enA, INPUT_PULLUP);
  pinMode(right_back_enB, INPUT_PULLUP);

  pinMode(left_front_enA, INPUT_PULLUP);
  pinMode(left_front_enB, INPUT_PULLUP);

  pinMode(left_back_enA, INPUT_PULLUP);
  pinMode(left_back_enB, INPUT_PULLUP);

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

  nh.getHardware()->setBaud(921600);
  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.advertise(lf_vel_pub);
  nh.advertise(rf_vel_pub);
  nh.advertise(lb_vel_pub);
  nh.advertise(rb_vel_pub);
  nh.subscribe(pidControlSubscriber);

}

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

    lf_vel_msg.data = lf_velocity;
    rf_vel_msg.data = rf_velocity;
    lb_vel_msg.data = lb_velocity;
    rb_vel_msg.data = rb_velocity;

    lf_vel_pub.publish(&lf_vel_msg);
    rf_vel_pub.publish(&rf_vel_msg);
    lb_vel_pub.publish(&lb_vel_msg);
    rb_vel_pub.publish(&rb_vel_msg);

  }

  nh.spinOnce();
}

void loop() {
  RPM_velocity_calc();
  PID_Execution();

}
