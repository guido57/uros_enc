#ifndef _ODOMETRY_H 
#define _ODOMETRY_H

#include <Arduino.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>

// Encoder pins
#define ENCL_A_PIN 26
#define ENCL_B_PIN 33
#define ENCR_A_PIN 13
#define ENCR_B_PIN 14

// Valid encoders state transitions
const uint8_t FORWARD_TRANSITIONS[4] = {2, 0, 3, 1};
const uint8_t BACKWARD_TRANSITIONS[4] = {1, 3, 0, 2};

extern rcl_publisher_t odom_publisher;

void update_odometry();
void publish_odometry();
void IRAM_ATTR encoderl_interrupt();
void IRAM_ATTR encoderr_interrupt();

#endif