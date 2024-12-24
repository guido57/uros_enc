#ifndef _ROS2_H
#define _ROS2_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#define RCCHECK(fn, msg)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("err=%d %s\r\n",temp_rc,msg);error_loop();}}
#define RCSOFTCHECK(fn, msg) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("err=%d %s\r\n",temp_rc,msg);}}

// ROS2 entities
extern rcl_subscription_t subscriber;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_timer_t timer;
extern std_msgs__msg__Int32 sub_msg;

#endif