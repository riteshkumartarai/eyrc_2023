/*
* Team Id: 1796
* Author List: Soumitra Naik
* Filename: espCode.ino
* Theme: HOLOGLYPH BOT
* Functions: setup(),loop()
* Global Variables: msg,pen_msg
*/
#include <stdio.h>
#include <ESP32Servo.h>              // Include ESP32 Servo library
#include <micro_ros_arduino.h>       // Include Micro-ROS Arduino library
#include <rcl/rcl.h>                 // Include ROS Client Library
#include <rcl/error_handling.h>      // Include ROS Client Library error handling
#include <rclc/rclc.h>               // Include ROS Client Library for C
#include <rclc/executor.h>           // Include ROS Client Library Executor
#include <geometry_msgs/msg/twist.h> // Include Twist message type
#include <std_msgs/msg/bool.h>       // Include Bool message type

// Define ROS subscription variables
rcl_subscription_t subscriber_cmd_vel;
geometry_msgs__msg__Twist msg;  //Global variable to store twist message {range: [-60,60]}
rcl_subscription_t subscriber_pen_down;
std_msgs__msg__Bool pen_msg;    //Global variable to store pen_down message {expected values: True,False}

// Define ROS executor
rclc_executor_t executor;

// Define ROS support variables
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//define servo names
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo_pen;

//define servo and LED pins
#define servo1_pin  25
#define servo2_pin  26
#define servo3_pin  27
#define servo_pen_pin  33
#define LED_PIN LED_BUILTIN

// Macro for error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

/*
* Function Name: error_loop
* Input: None
* Output: None
* Logic: This function toggles the state of the built-in LED in an infinite loop, providing visual indication of an error.
* Example Call: error_loop();
*/
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

/*
* Function Name: cmd_vel_callback
* Input: const void * msgin (pointer to the received message)
* Output: None
* Logic: This function processes the received Twist message from the cmd_vel topic and controls the servos accordingly.
* Example Call: cmd_vel_callback(&msg);
*/
void cmd_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // Your processing logic for Twist messages goes here
  servo1.write(msg->linear.x + 90);
  servo2.write(msg->linear.y + 90);
  servo3.write(msg->linear.z + 90);
}

/*
* Function Name: pen_down_callback
* Input: const void * msgin (pointer to the received message)
* Output: None
* Logic: This function processes the received Bool message from the pen_down topic and controls the pen servo accordingly.
* Example Call: pen_down_callback(&pen_msg);
*/
void pen_down_callback(const void * msgin)
{
  const std_msgs__msg__Bool * pen_msg = (const std_msgs__msg__Bool *)msgin;
  // Your processing logic for Bool messages goes here
  // Process msg_pen_down (bool) if needed
  if (pen_msg->data) {
    servo_pen.write(180);
  } else {
    servo_pen.write(140);
  }
}

/*
* Function Name: setup
* Input: None
* Output: None
* Logic: This function is called once when the microcontroller starts. It initializes ROS, servos, and other peripherals.
* Example Call: setup();
*/
void setup() {
  // Initialize Wi-Fi transports for micro-ROS
  set_microros_wifi_transports("subun", "123456789", "192.168.168.44", 8888);

  // Initialize serial communication
  Serial.begin(9600);

  // Attach servos to pins
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  servo_pen.attach(servo_pen_pin);

  // Set LED pin mode and turn off LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  // Delay for stabilization
  delay(2000);

  // Initialize ROS support and node
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "subscriber_node_2", "", &support));
  
  // Initialize ROS subscriptions for cmd_vel and pen_down topics
  RCCHECK(rclc_subscription_init_default(
    &subscriber_cmd_vel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel/bot2"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_pen_down,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/pen2_down"));

  // Initialize ROS executor and add subscriptions
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_cmd_vel, &msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_pen_down, &pen_msg, &pen_down_callback, ON_NEW_DATA));
}

/*
* Function Name: loop
* Input: None
* Output: None
* Logic: This function is called repeatedly in an infinite loop after the setup() function. It spins the ROS executor to process incoming messages.
* Example Call: loop();
*/
void loop() {
  RCCHECK(rclc_executor_spin_some(&executor,RCL_MS_TO_NS(1000000)));
}
