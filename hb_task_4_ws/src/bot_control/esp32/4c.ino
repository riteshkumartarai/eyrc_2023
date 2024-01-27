#include <ESP32Servo.h>


#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>  // Include Twist message type


rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;  //  Twist
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
// Assigning servo names
Servo servo1; 
Servo servo2; 
Servo servo3; 
// Defining pins
#define servo1_pin  25
#define servo2_pin  26
#define servo3_pin  27
#define LED_PIN LED_BUILTIN

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;  // Change Int32 to Twist
  // Your processing logic for Twist messages goes here
}

void setup() {
  set_microros_wifi_transports("Subun", "123456789", "192.168.144.44", 8888);
  Serial.begin(9600);
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "subscriber_node_3", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),  
    "/cmd_vel/bot3"));  // subscribing topic for Twist messages

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  // Your additional logic goes here
  servo1.write(msg.linear.x+90);
  servo2.write(msg.linear.y+90);
  servo3.write(msg.linear.z+90);
  
}
