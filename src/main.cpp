// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
//#include "/home/guido/Documents/PlatformIO/Projects/uros_demo/.pio/libdeps/esp32dev/micro_ros_platformio/platform_code/arduino/wifi/micro_ros_transport.h"
// #include <micro_ros_transport.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_GIGA) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL) && !defined(ARDUINO_UNOR4_WIFI) && !defined(ARDUINO_OPTA)
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi 
#endif

// set ssid and pass for the wifi and ROS2 agent address and port
// e.g.
// String ssid = "ssid_name"
// String pass = "ssid_password"
// IPAddress ros2_agent_ipa = IPAddress(192,168,1,13);
// int ros2_agent_port = 8888;
#include "credentials.h"

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 pub_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;
std_msgs__msg__Int32 sub_msg;

#if defined(LED_BUILTIN)
  #define LED_PIN LED_BUILTIN
#else
  #define LED_PIN 2
#endif

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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL),"timer is null!");
    pub_msg.data++;
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

  delay(2000);

  allocator = rcl_get_default_allocator();

  printf("create init_options...\r\n");
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator),"rclc_support_init error!");

  // create node
  printf("create node...\r\n");
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support),"rclc_mode_init_default error!");

  // create publisher
  printf("create publisher...\r\n");
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "topic_name"), "rclc_publisher_init_best_effort error!");

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

  pub_msg.data = 0;
  printf("end of setup!\r\n");
  
}

void loop() {
    
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)),"rclc_executor_spin_some error");
    if(WiFi.status() != WL_CONNECTED)
      my_connect();
}