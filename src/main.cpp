// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
//#include <rclc_parameter/rclc_parameter.h>
#include <nav_msgs/msg/odometry.h>

#include <std_msgs/msg/int32.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_GIGA) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL) && !defined(ARDUINO_UNOR4_WIFI) && !defined(ARDUINO_OPTA)
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi 
#endif

#include "credentials.h"

// Odometry variables
volatile int enc_r_position = 0;
volatile int enc_l_position = 0;
volatile int enc_r_errors = 0;
volatile int enc_l_errors = 0;
float x = 0.0, y = 0.0, theta = 0.0;

// ROS2 entities
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;
rcl_subscription_t subscriber;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;
std_msgs__msg__Int32 sub_msg;

#define LED_PIN 2

// Encoder pins
#define ENCL_A_PIN 26
#define ENCL_B_PIN 33
#define ENCR_A_PIN 13
#define ENCR_B_PIN 14
// Left Motor PWM pins 
# define PWW_LEFT_1_PIN 4
# define PWW_LEFT_2_PIN 12
// Right Motor PWM pins 
# define PWW_RIGHT_1_PIN 27
# define PWW_RIGHT_2_PIN 32

// Default parameters
float wheel_radius = 0.03625;   // in meters
float wheel_base = 0.136;       // in meters
int ticks_per_revolution = 2976;

// Encoder states
volatile uint8_t encr_state = 0;
volatile uint8_t encl_state = 0;

// Valid encoders state transitions
const uint8_t FORWARD_TRANSITIONS[4] = {2, 0, 3, 1};
const uint8_t BACKWARD_TRANSITIONS[4] = {1, 3, 0, 2};

#define RCCHECK(fn, msg)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("err=%d %s\r\n",temp_rc,msg);error_loop();}}
#define RCSOFTCHECK(fn, msg) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("err=%d %s\r\n",temp_rc,msg);}}

struct my_micro_ros_agent_locator {
    IPAddress address;
    int port;
};

void my_connect(){
    WiFi.disconnect(true);   // Reset Wi-Fi
    WiFi.mode(WIFI_STA);     // Set to Station mode
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

static void my_set_microros_wifi_transports(String ssid, String pass, IPAddress agent_ip, uint16_t agent_port){
    
    my_connect();
    
    static struct my_micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

#define MAX_STATES 100
int enc1_states[MAX_STATES];
int enc1_new[MAX_STATES];
int count_states_1 = 0;
void IRAM_ATTR encoderr_interrupt() {
  uint8_t new_state = (digitalRead(ENCR_A_PIN) << 1) | digitalRead(ENCR_B_PIN);
  if(count_states_1 < MAX_STATES){
    enc1_new[count_states_1] = new_state;
    enc1_states[count_states_1++] = encr_state;
  }
  //Serial.printf("enc1_state=%d new_state=%d\r\n", enc1_state,new_state);
  if (new_state == FORWARD_TRANSITIONS[encr_state]) {
    enc_r_position++;
  } else if (new_state == BACKWARD_TRANSITIONS[encr_state]) {
    enc_r_position--;
  } else {
    enc_r_errors++;
  }
  encr_state = new_state;
}

int enc2_states[MAX_STATES];
int enc2_new[MAX_STATES];
int count_states_2 = 0;
void IRAM_ATTR encoderl_interrupt() {
  uint8_t new_state = (digitalRead(ENCL_A_PIN) << 1) | digitalRead(ENCL_B_PIN);
  if(count_states_2 < MAX_STATES){
    enc2_new[count_states_2] = new_state;
    enc2_states[count_states_2++] = encl_state;
  }
  if (new_state == FORWARD_TRANSITIONS[encl_state]) {
    enc_l_position++;
  } else if (new_state == BACKWARD_TRANSITIONS[encl_state]) {
    enc_l_position--;
  } else {
    enc_l_errors++;
  }
  encl_state = new_state;
}

void update_odometry() {
  float left_distance = (2 * PI * wheel_radius * enc_r_position) / ticks_per_revolution;
  float right_distance = (2 * PI * wheel_radius * enc_l_position) / ticks_per_revolution;
  float distance = (left_distance + right_distance) / 2.0;
  float delta_theta = (right_distance - left_distance) / wheel_base;

  theta += delta_theta;
  x += distance * cos(theta);
  y += distance * sin(theta);
  printf("update_odometry: x=%f y=%f theta=%f enc1_pos=%d enc1_errors=%d enc2_pos=%d enc2_errors=%d\r\n",x,y,theta,enc_r_position, enc_r_errors, enc_l_position,enc_l_errors );
  
  // Reset encoder positions for next calculation
  enc_r_position = 0;
  enc_l_position = 0;
}

void publish_odometry() {
  update_odometry();

  odom_msg.header.stamp.sec = millis() / 1000;
  odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
  odom_msg.header.frame_id.data = const_cast<char*>("odom");
  odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL),"rcl_publish /odom error");
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    publish_odometry();
  }
}

void subscription_callback(const void * msg_in)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msg_in;
  printf("Received message: %d\r\n", msg->data);
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void setup() {
  Serial.begin(115200);
  printf("setup ...\r\n");
  my_set_microros_wifi_transports(ssid, pass, ros2_agent_ipa, ros2_agent_port);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(ENCR_A_PIN, INPUT_PULLDOWN);
  pinMode(ENCR_B_PIN, INPUT_PULLDOWN);
  pinMode(ENCL_A_PIN, INPUT_PULLDOWN);
  pinMode(ENCL_B_PIN, INPUT_PULLDOWN);

  attachInterrupt(ENCR_A_PIN, encoderr_interrupt, CHANGE);
  attachInterrupt(ENCR_B_PIN, encoderr_interrupt, CHANGE);
  attachInterrupt(ENCL_A_PIN, encoderl_interrupt, CHANGE);
  attachInterrupt(ENCL_B_PIN, encoderl_interrupt, CHANGE);

  delay(2000);

  allocator = rcl_get_default_allocator();

  printf("create init_options...\r\n");
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator),"rclc_support_init error!");

  // create node
  printf("create node...\r\n");
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support),"rclc_mode_init_default error!");

  // create /odom publisher
  printf("create /odom publisher...\r\n");
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom"), "rclc_publisher_init_best_effort error!");

  printf("create subscriber...\r\n");
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "sub_topic_name"), "rclc_subscription_init_default error!");

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback),"rclc_timer_init_default error!");

  int a = RCL_RET_INVALID_ARGUMENT;  

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator),"rclc_executor_init error");
  RCCHECK(rclc_executor_add_timer(&executor, &timer),"rclc_executor_add_timer error");
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, subscription_callback, ON_NEW_DATA),"rclc_executor_add_subscription error!");

  printf("end of setup!\r\n");
}

void loop() {
    
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)),"rclc_executor_spin_some error");
    if(WiFi.status() != WL_CONNECTED)
      my_connect();
}