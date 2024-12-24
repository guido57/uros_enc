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
#include "rpLidar.h"
#include "rpLidarTypes.h"

rcl_subscription_t subscriber;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;
rcl_timer_t timer_scan;
rcl_publisher_t scan_publisher;
std_msgs__msg__Int32 sub_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__LaserScan scan_msg;

#define LED_PIN 2

// lidar Parameters and lidar object
#define SCAN_PUBLISH_INTERVAL 100  // Milliseconds
const int MAX_SCAN_POINTS = 550;
// rpLidar lidar(&Serial2, 460800);
rpLidar * lidar;
static float ranges[MAX_SCAN_POINTS];
static float intensities[MAX_SCAN_POINTS];

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

void motors_control(){
float linearVelocity;
  float angularVelocity;
  //linear velocity and angular velocity send cmd_vel topic
  linearVelocity  = cmd_vel_msg.linear.x;
  angularVelocity = cmd_vel_msg.angular.z;
  //linear and angular velocities are converted to leftwheel and rightwheel velocities
  float vL = linearVelocity - angularVelocity * wheel_base / 2;
  float vR = linearVelocity + angularVelocity * wheel_base / 2;
  //current wheel rpm is calculated
  float currentRpmL = leftWheel.getRpm(enc_l_total);
  float currentRpmR = rightWheel.getRpm(enc_r_total);
  // current wheel speed (m/s) is calculated
  float currentmsL = currentRpmL * 2 * PI * wheel_radius / 60;
  float currentmsR = currentRpmR * 2 * PI * wheel_radius / 60;

  //pid controlled is used for generating the pwm signal
  float actuating_signal_LW = leftWheel.pid(vL, currentmsL);
  float actuating_signal_RW = rightWheel.pid(vR, currentmsR);
  printf("lv=%f av=%f vL=%f vR=%f msL=%f msR=%f LW=%f RW=%f                     \r",
        linearVelocity, angularVelocity, vL, vR, currentmsL, currentmsR, actuating_signal_LW, actuating_signal_RW
  );
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

void publish_scan(int count) {
  scan_msg.header.stamp.sec = millis() / 1000;
  scan_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

  scan_msg.angle_min = 0.0;
  scan_msg.angle_max = 2 * M_PI;
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / count;
  scan_msg.range_min = 0.05;  // Minimum valid distance in meters
  scan_msg.range_max = 12.0; // Maximum valid distance in meters

  for (int i = 0; i < count; i++) {
    float angle = (lidar->DataBuffer[i].angle_high * 128 + lidar->DataBuffer[i].angle_low / 2) / 64.0 * DEG_TO_RAD;
    int distance = lidar->DataBuffer[i].distance_high * 256 + lidar->DataBuffer[i].distance_low;
    int quality = lidar->DataBuffer[i].quality / 4;

    // Store data in scan message
    scan_msg.ranges.data[i] = (distance > 0) ? distance / 1000.0 : INFINITY;  // Convert mm to meters
    scan_msg.intensities.data[i] = quality;
  }

  scan_msg.ranges.size = count;
  scan_msg.intensities.size = count;

  // Publish the scan message
  RCSOFTCHECK(rcl_publish(&scan_publisher, &scan_msg, NULL), "error publishing /scan");
}

void scan_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
 if (timer != NULL )  {


    //publish_scan(count); // Publish the scan data to /scan topic

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
  
  lidar = new rpLidar(&Serial2, 460800);

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

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &scan_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "/scan"),"rclc_publisher_init_best_effort /scan error!");

  // Initialize scan message
  scan_msg.header.frame_id.data = (char *)"lidar_frame";
  scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
  scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;
  scan_msg.ranges.capacity = MAX_SCAN_POINTS;
  scan_msg.ranges.data = ranges;
  scan_msg.intensities.capacity = MAX_SCAN_POINTS;
  scan_msg.intensities.data = intensities;

  // Setup lidar
  lidar->resetDevice(); // Reset the device to ensure good status
  stDeviceStatus_t sdst = lidar->getDeviceHealth();
  Serial.printf("sdst.errorCode_high=%d  sdst.errorCode_low=%d sdst.status=%d\r\n", sdst.errorCode_high, sdst.errorCode_low, sdst.status);

  //lidar.setAngleOfInterest(0, 360); // Set field of view
  bool ret = lidar->start(standard); // Start standard scan
  if (ret)
    Serial.println("rplidar C1 started correctly!");
  else
    Serial.println("Error starting rplidar C1");
  
  // create 100 msecs timer
  printf("Create /scan timer...\r\n");
  const unsigned int timer_scan_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer_scan,
    &support,
    RCL_MS_TO_NS(timer_scan_timeout),
    scan_timer_callback),"rclc_timer_init_default scan_timer_callback error!");

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
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    motor_timer_callback),"rclc_timer_init_default error!");

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator),"rclc_executor_init error");
  RCCHECK(rclc_executor_add_timer(&executor, &timer),"rclc_executor_add_timer motor error");
  RCCHECK(rclc_executor_add_timer(&executor, &timer_scan),"rclc_executor_add_timer scan error");
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, sub_cmd_vel_callback, ON_NEW_DATA),"rclc_executor_add_subscription error!");

  printf("end of setup!\r\n");
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)),"rclc_executor_spin_some error");
}