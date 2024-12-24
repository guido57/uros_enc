#include "odometry.h"
#include "ros2.h"

// Default parameters
float wheel_radius = 0.03625;   // in meters
float wheel_base = 0.136;       // in meters
int ticks_per_revolution = 2976;

// Odometry variables
int enc_r_total = 0;
int enc_l_total = 0;
volatile int enc_r_position = 0;
volatile int enc_l_position = 0;
volatile int enc_r_errors = 0;
volatile int enc_l_errors = 0;
float x = 0.0, y = 0.0, theta = 0.0;

// Encoder states
volatile uint8_t encr_state = 0;
volatile uint8_t encl_state = 0;

nav_msgs__msg__Odometry odom_msg;
rcl_publisher_t odom_publisher;


void update_odometry() {
  float left_distance = (2 * PI * wheel_radius * enc_r_position) / ticks_per_revolution;
  float right_distance = (2 * PI * wheel_radius * enc_l_position) / ticks_per_revolution;
  float distance = (left_distance + right_distance) / 2.0;
  float delta_theta = (right_distance - left_distance) / wheel_base;

  theta += delta_theta;
  x += distance * cos(theta);
  y += distance * sin(theta);
  //printf("update_odometry: x=%f y=%f theta=%f enc_r_pos=%d enc_r_tot=%d enc1_errors=%d enc_l_pos=%d enc_l_tot=%d enc2_errors=%d                     \r",
  //      x,y,theta,enc_r_position, enc_r_total, enc_r_errors,  enc_l_position, enc_l_total, enc_l_errors );
  
  // Reset encoder positions for next calculation
  enc_r_position = 0;
  enc_l_position = 0;
}

void publish_odometry() {

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

void IRAM_ATTR encoderr_interrupt() {
  uint8_t new_state = (digitalRead(ENCR_A_PIN) << 1) | digitalRead(ENCR_B_PIN);
  //Serial.printf("enc1_state=%d new_state=%d\r\n", enc1_state,new_state);
  if (new_state == FORWARD_TRANSITIONS[encr_state]) {
    enc_r_position++;
    enc_r_total ++;
  } else if (new_state == BACKWARD_TRANSITIONS[encr_state]) {
    enc_r_position--;
    enc_r_total --;
  } else {
    enc_r_errors++;
  }
  encr_state = new_state;
}

void IRAM_ATTR encoderl_interrupt() {
  uint8_t new_state = (digitalRead(ENCL_A_PIN) << 1) | digitalRead(ENCL_B_PIN);
  if (new_state == FORWARD_TRANSITIONS[encl_state]) {
    enc_l_position++;
    enc_l_total++;
  } else if (new_state == BACKWARD_TRANSITIONS[encl_state]) {
    enc_l_position--;
    enc_l_total--;
  } else {
    enc_l_errors++;
  }
  encl_state = new_state;
}
