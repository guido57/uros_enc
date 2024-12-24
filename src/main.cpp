// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/laser_scan.h>

#if !defined(ESP32) 
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi 
#endif

#include "credentials.h"
#include "odometry.h"
#include "ros2.h"
#include "MotorController.h"
#include "RplidarC1.h"

rcl_subscription_t subscriber;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t odom_pid_timer;
rcl_timer_t lidar_timer;
rcl_publisher_t lidar_publisher;
std_msgs__msg__Int32 sub_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__LaserScan scan_msg;

#define LED_PIN 2

// Lidar object
RplidarC1 lidar;

// Left Motor PWM pins 
# define PWM_LEFT_1_PIN 4
# define PWM_LEFT_2_PIN 12
// Right Motor PWM pins 
# define PWM_RIGHT_1_PIN 27
# define PWM_RIGHT_2_PIN 32

//creating objects for right wheel and left wheel
//encoder value per revolution of left wheel and right wheel
int tickPerRevolution_LW = 2976;
int tickPerRevolution_RW = 2976;

extern float wheel_radius; // = 0.03625;   // in meters
extern float wheel_base; // = 0.136;       // in meters

// total ticks counters from odometry
extern int enc_r_total;
extern int enc_l_total;

//pid constants of left wheel
float kp_l = 200;  // it was 2.0
float ki_l = 800;    // it was 5.0
float kd_l = 50; // it was 0.1
//pid constants of right wheel
float kp_r = 200; // it was 2.0
float ki_r = 800;   // it was 5.0
float kd_r = 50; // it was 0.1

//pwm parameters setup
const int freq = 30000;
const int resolution = 8;

MotorController leftWheel(PWM_LEFT_1_PIN, PWM_LEFT_2_PIN, tickPerRevolution_LW);
MotorController rightWheel(PWM_RIGHT_1_PIN, PWM_RIGHT_2_PIN, tickPerRevolution_RW);

float linearVelocity, angularVelocity;
float vL, vR;
float currentRpmL, currentRpmR;
float currentmsL, currentmsR;
float actuating_signal_LW, actuating_signal_RW;
void motors_control(){
  //linear velocity and angular velocity send cmd_vel topic
  linearVelocity  = cmd_vel_msg.linear.x;
  angularVelocity = cmd_vel_msg.angular.z;
  //linear and angular velocities are converted to leftwheel and rightwheel velocities
  vL = linearVelocity - angularVelocity * wheel_base / 2;
  vR = linearVelocity + angularVelocity * wheel_base / 2;
  //current wheel rpm is calculated
  currentRpmL = leftWheel.getRpm(enc_l_total);
  currentRpmR = rightWheel.getRpm(enc_r_total);
  // current wheel speed (m/s) is calculated
  currentmsL = currentRpmL * 2 * PI * wheel_radius / 60;
  currentmsR = currentRpmR * 2 * PI * wheel_radius / 60;

  //pid controlled is used for generating the pwm signal
  actuating_signal_LW = leftWheel.pid(vL, currentmsL);
  actuating_signal_RW = rightWheel.pid(vR, currentmsR);
  // printf("lv=%.3f av=%.3f vL=%.2f vR=%.3f msL=%.3f msR=%.3f LW=%.0f RW=%f.0                     \r",
  //       linearVelocity, angularVelocity, vL, vR, currentmsL, currentmsR, actuating_signal_LW, actuating_signal_RW
  //);
  if (vL == 0 && vR == 0) { 
    leftWheel.stop(0,1);
    rightWheel.stop(2,3);
    actuating_signal_LW = 0;
    actuating_signal_RW = 0;
  } else {
    leftWheel.moveBase(actuating_signal_LW, 130, 0, 1);
    rightWheel.moveBase(actuating_signal_RW,130, 2, 3);
  }
}

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

static void my_set_microros_wifi_transports(){
    
    my_connect();

    struct my_micro_ros_agent_locator {
      IPAddress address;
      int port;
    } static locator;
    locator.address = ros2_agent_ipa;
    locator.port = ros2_agent_port;

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

void motor_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    motors_control();
    update_odometry();
    publish_odometry();
  }
}

unsigned long total_loop_time = 0L;
float loop_period = 0.0;
void lidar_loop(){

    unsigned long uart_elapsed = millis();
    int count = lidar.uartRx();
    uart_elapsed = millis() - uart_elapsed;

    unsigned long process_elapsed = millis();
    lidar.processFrame(count);
    process_elapsed = millis()-process_elapsed;

    unsigned long publish_elapsed = millis();
    rcl_ret_t ret_pub = rcl_publish(&lidar_publisher, &lidar.scan_msg, NULL);
    publish_elapsed = millis() - publish_elapsed;

    if(ret_pub != RCL_RET_OK){
        printf("rcl_publish returned %d\r\n", ret_pub);
        esp_restart();
    }

    // calculate loop period  
    total_loop_time = millis()-total_loop_time;
    float total_loop_time_f = (float) total_loop_time;
    loop_period = loop_period*0.9 + total_loop_time_f*0.1;
    
    Serial.printf("lv=%.3f av=%.3f vL=%.2f vR=%.3f msL=%.3f msR=%.3f LW=%.0f RW=%.0f  %dpts:%lums-FrameProc:%lums-FramePub:%lums-LidarLoop:%lums-LidarFreq=%.1fHz-Serial2.available=%d\r\n",
        linearVelocity, angularVelocity, vL, vR, currentmsL, currentmsR, actuating_signal_LW, actuating_signal_RW,
        count, uart_elapsed, process_elapsed, publish_elapsed, total_loop_time, 1000.0/loop_period, Serial2.available()
     );
    total_loop_time = millis();
}

void lidar_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL )  {

    unsigned long lidar_timer_cb_elapsed = millis(); 
    lidar_loop();
    // printf("\r\n%lu lidar_timer_callback took %lu millis. Serial2aval=%d\r\n", 
    //       millis(), millis()-lidar_timer_cb_elapsed, Serial2.available());
  }
}

void sub_cmd_vel_callback(const void * msg_in)
{
  const geometry_msgs__msg__Twist *msg_conv = (const geometry_msgs__msg__Twist *)msg_in;
  cmd_vel_msg = * msg_conv;
 
  //printf("Received message: %f\r\n", msg_conv->linear.x);
}

void setup() {
  Serial.begin(115200);
  printf("setup ...\r\n");
  
  lidar.begin();
  delay(1000);
  lidar.resetLidar();
  delay(800);
  lidar.startLidar();
  my_set_microros_wifi_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Set Encoders pins
  pinMode(ENCR_A_PIN, INPUT_PULLDOWN);
  pinMode(ENCR_B_PIN, INPUT_PULLDOWN);
  pinMode(ENCL_A_PIN, INPUT_PULLDOWN);
  pinMode(ENCL_B_PIN, INPUT_PULLDOWN);
  // Attach Encoders Interrupts
  attachInterrupt(ENCR_A_PIN, encoderr_interrupt, CHANGE);
  attachInterrupt(ENCR_B_PIN, encoderr_interrupt, CHANGE);
  attachInterrupt(ENCL_A_PIN, encoderl_interrupt, CHANGE);
  attachInterrupt(ENCL_B_PIN, encoderl_interrupt, CHANGE);

  //initializing the pid constants
  leftWheel.initPID(kp_l, ki_l, kd_l);
  rightWheel.initPID(kp_r, ki_r, kd_r);

  //initializing pwm signal parameters
  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);
  ledcSetup(2, freq, resolution);
  ledcSetup(3, freq, resolution);
  ledcAttachPin(PWM_LEFT_1_PIN, 0);
  ledcAttachPin(PWM_LEFT_2_PIN, 1);
  ledcAttachPin(PWM_RIGHT_1_PIN, 2);
  ledcAttachPin(PWM_RIGHT_2_PIN, 3);
  
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
    "/odom"), "rclc_publisher_init_default /odom error!");

  // Create /scan publisher
  printf("create /scan publisher...\r\n");
  RCCHECK(rclc_publisher_init_default(
    &lidar_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "/scan"),"rclc_publisher_init_best_effort /scan error!");
  
  // create 100 msecs timer
  printf("Create /scan timer...\r\n");
  const unsigned int timer_scan_timeout = 70;
  RCCHECK(rclc_timer_init_default(
    &lidar_timer,
    &support,
    RCL_MS_TO_NS(timer_scan_timeout),
    lidar_timer_callback),"rclc_timer_init_default scan_timer_callback error!");

  // Initialize subscriber
  printf("create /cmd_vel subscriber...\r\n");
  RCCHECK(  rclc_subscription_init_default(
                  &subscriber,
                  &node,
                  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                  "cmd_vel"),
            "rcl_subscription_init error"
  );

  // Initialize message
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);

  // create 20 msecs timer
  printf("create motor timer...\r\n");
  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(
    &odom_pid_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    motor_timer_callback),"rclc_timer_init_default error!");

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator),"rclc_executor_init error");
  RCCHECK(rclc_executor_add_timer(&executor, &odom_pid_timer),"rclc_executor_add_timer motor error");
  RCCHECK(rclc_executor_add_timer(&executor, &lidar_timer),"rclc_executor_add_timer scan error");
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, sub_cmd_vel_callback, ON_NEW_DATA),"rclc_executor_add_subscription error!");

  printf("end of setup!\r\n");
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)),"rclc_executor_spin_some error");
}