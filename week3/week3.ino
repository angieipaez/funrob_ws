#include <micro_ros_arduino.h>
#include <stdio.h> 
#include <rcl/rcl.h> 
#include <rcl/error_handling.h> 
#include <rclc/rclc.h>
#include <rclc/executor.h> 
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

// ROS structures and variables
rcl_subscription_t subscriber;
rcl_publisher_t raw_pot_pub;
rcl_publisher_t voltage_pub;
std_msgs_msg_Float32 msg_float_1;
std_msgs_msg_Float32 msg_float_2;
std_msgs_msg_Int32 msg_int;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer1;
rcl_timer_t timer2;

// Pin definitions and constants
int pot = 0;
float voltage = 0;
float pwm = 0;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
#define PWM_PIN 15
#define redLedPin 13
#define blueLedPin 12
#define POTENTIOMETER_PIN 36

// Macros for error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Function to handle errors
void error_loop(){
  while(1){
    digitalWrite(redLedPin, !digitalRead(redLedPin));
    delay(100);
  }
}

// Callback function for timer1
void timer1_callback(rcl_timer_t * timer1, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time); 
  if (timer1 != NULL) {
    pot = analogRead(POTENTIOMETER_PIN); // Read potentiometer value
  }
  float pwm_value = (float)(pwm * 255/100); // Calculate PWM value
  ledcWrite(PWM_CHANNEL, abs(pwm_value)); // Write PWM value to the pin
}

// Callback function for timer2
void timer2_callback(rcl_timer_t * timer2, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time); 
  if (timer2 != NULL) {
    msg_int.data = pot; // Assign potentiometer value to ROS message
    RCSOFTCHECK(rcl_publish(&raw_pot_pub, &msg_int, NULL)); // Publish potentiometer value
    voltage = pot * (3.3 / 4095); // Convert potentiometer value to voltage
    msg_float_1.data = voltage; // Assign voltage value to ROS message
    RCSOFTCHECK(rcl_publish(&voltage_pub, &msg_float_1, NULL)); // Publish voltage value
  }
}

// Callback function for subscription to PWM duty cycle
void subscription_callback(const void * msgin)
{  
  const std_msgs_msgFloat32 * msg = (const std_msgsmsg_Float32 *)msgin; 
  pwm = msg->data; // Retrieve PWM duty cycle from the message
}

// Setup function (runs once at the beginning)
void setup() {
  set_microros_transports();
  
  pinMode(redLedPin, OUTPUT); 
  digitalWrite(redLedPin, HIGH); 
  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL); 
  
  delay(2000); 
  
  allocator = rcl_get_default_allocator(); 
  
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); 
  
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));
  
  RCCHECK(rclc_publisher_init_default(
    &raw_pot_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/raw_pot")); 
  
  RCCHECK(rclc_publisher_init_default(
    &voltage_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/voltage"));
  
  const unsigned int timer1_timeout = 10; 
  RCCHECK(rclc_timer_init_default(
    &timer1,
    &support,
    RCL_MS_TO_NS(timer1_timeout),
    timer1_callback)); // Initialize timer1
  
  const unsigned int timer2_timeout = 100; 
  RCCHECK(rclc_timer_init_default(
    &timer2,
    &support,
    RCL_MS_TO_NS(timer2_timeout),
    timer2_callback));
  
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/pwm_duty_cycle")); 
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator)); 
  RCCHECK(rclc_executor_add_timer(&executor, &timer1)); 
  RCCHECK(rclc_executor_add_timer(&executor, &timer2)); 
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_float_2, &subscription_callback, ON_NEW_DATA));
}

// Loop function (runs repeatedly)
void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); 
}
