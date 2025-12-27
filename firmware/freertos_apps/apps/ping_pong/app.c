// ===================== micro-ROS core includes =====================

// Core ROS Client Library (C API)
// Provides node, publisher, subscription primitives
#include <rcl/rcl.h>

// Provides error codes and error handling utilities
#include <rcl/error_handling.h>

// rclc = convenience layer on top of rcl
// Simplifies node, publisher, subscription, timer creation
#include <rclc/rclc.h>

// Executor handles dispatching callbacks (timers + subscriptions)
#include <rclc/executor.h>

// ===================== Message definition =====================

// std_msgs/Header message definition
// Contains:
//   builtin_interfaces/Time stamp
//   string frame_id
#include <std_msgs/msg/header.h>

// ===================== Standard C / POSIX =====================

// printf
#include <stdio.h>

// usleep (microsecond sleep)
#include <unistd.h>

// clock_gettime (timestamp generation)
#include <time.h>

// ===================== ESP32 / FreeRTOS specific =====================

// These headers exist only when compiling for ESP32
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"  // FreeRTOS kernel
#include "freertos/task.h"      // FreeRTOS task API
#endif

// ===================== Constants =====================

// Fixed buffer size for ROS string fields
// micro-ROS DOES NOT allocate memory dynamically for strings
#define STRING_BUFFER_LEN 50

// ===================== Error handling macros =====================

// RCCHECK: hard failure
// If any micro-ROS call fails, print error and kill the FreeRTOS task
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
    vTaskDelete(NULL); \
  } \
}

// RCSOFTCHECK: soft failure
// Logs error but continues execution
#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
  } \
}

// ===================== ROS entities (global lifetime) =====================

// Publisher that sends ping messages
rcl_publisher_t ping_publisher;

// Publisher that sends pong responses
rcl_publisher_t pong_publisher;

// Subscriber that receives ping messages
rcl_subscription_t ping_subscriber;

// Subscriber that receives pong messages
rcl_subscription_t pong_subscriber;

// ===================== Message instances =====================

// Message used to receive ping messages
std_msgs__msg__Header incoming_ping;

// Message used to send ping messages
std_msgs__msg__Header outcoming_ping;

// Message used to receive pong messages
std_msgs__msg__Header incoming_pong;

// ===================== State variables =====================

// Unique ID for this ESP32 instance
int device_id;

// Random sequence number for each ping
int seq_no;

// Number of pong replies received for current ping
int pong_count;

// ===================== Timer callback =====================
// Called every 2 seconds by the executor

void ping_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  // Macro to silence unused parameter warning
  RCLC_UNUSED(last_call_time);

  // Ensure timer is valid
  if (timer != NULL) {

    // Generate random sequence number
    seq_no = rand();

    // Write sequence + device_id into frame_id buffer
    sprintf(outcoming_ping.frame_id.data, "%d_%d", seq_no, device_id);

    // IMPORTANT: manually update string size
    outcoming_ping.frame_id.size = strlen(outcoming_ping.frame_id.data);

    // Get current system time (ESP32 system clock)
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    // Fill ROS timestamp
    outcoming_ping.stamp.sec = ts.tv_sec;
    outcoming_ping.stamp.nanosec = ts.tv_nsec;

    // Reset pong counter for this ping
    pong_count = 0;

    // Publish ping message
    rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL);

    // Debug output
    printf("Ping send seq %s\n", outcoming_ping.frame_id.data);
  }
}

// ===================== Ping subscription callback =====================
// Triggered when ANY node publishes to /microROS/ping

void ping_subscription_callback(const void * msgin)
{
  // Cast incoming generic pointer to Header message
  const std_msgs__msg__Header * msg =
    (const std_msgs__msg__Header *)msgin;

  // Ignore our own ping messages
  if (strcmp(outcoming_ping.frame_id.data, msg->frame_id.data) != 0) {

    // Print received ping ID
    printf("Ping received with seq %s. Answering.\n", msg->frame_id.data);

    // Send pong response using the same message content
    rcl_publish(&pong_publisher, (const void*)msg, NULL);
  }
}

// ===================== Pong subscription callback =====================
// Triggered when ANY node publishes to /microROS/pong

void pong_subscription_callback(const void * msgin)
{
  // Cast incoming pointer
  const std_msgs__msg__Header * msg =
    (const std_msgs__msg__Header *)msgin;

  // Only count pong responses to our own ping
  if (strcmp(outcoming_ping.frame_id.data, msg->frame_id.data) == 0) {
    pong_count++;
    printf("Pong for seq %s (%d)\n",
           msg->frame_id.data, pong_count);
  }
}

// ===================== Entry point (FreeRTOS task) =====================
// This is equivalent to main() in ESP-IDF firmware

void appMain(void *argument)
{
  // Get default memory allocator for micro-ROS
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // Support object holds context, allocator, arguments
  rclc_support_t support;

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // ===================== Node creation =====================

  rcl_node_t node;

  // Create ROS node named "pingpong_node"
  RCCHECK(rclc_node_init_default(
    &node,
    "pingpong_node",
    "",
    &support
  ));

  // ===================== Publishers =====================

  // Reliable ping publisher
  RCCHECK(rclc_publisher_init_default(
    &ping_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
    "/microROS/ping"
  ));

  // Best-effort pong publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &pong_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
    "/microROS/pong"
  ));

  // ===================== Subscriptions =====================

  // Best-effort ping subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &ping_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
    "/microROS/ping"
  ));

  // Best-effort pong subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &pong_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
    "/microROS/pong"
  ));

  // ===================== Timer =====================

  rcl_timer_t timer;

  // Create timer that fires every 2000 ms
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(2000),
    ping_timer_callback
  ));

  // ===================== Executor =====================

  rclc_executor_t executor;

  // Executor can handle 3 handles:
  // 1 timer + 2 subscriptions
  RCCHECK(rclc_executor_init(
    &executor,
    &support.context,
    3,
    &allocator
  ));

  // Register timer callback
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Register ping subscription callback
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &ping_subscriber,
    &incoming_ping,
    &ping_subscription_callback,
    ON_NEW_DATA
  ));

  // Register pong subscription callback
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &pong_subscriber,
    &incoming_pong,
    &pong_subscription_callback,
    ON_NEW_DATA
  ));

  // ===================== Message memory allocation =====================

  // Buffer for outgoing ping frame_id
  char outcoming_ping_buffer[STRING_BUFFER_LEN];
  outcoming_ping.frame_id.data = outcoming_ping_buffer;
  outcoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

  // Buffer for incoming ping frame_id
  char incoming_ping_buffer[STRING_BUFFER_LEN];
  incoming_ping.frame_id.data = incoming_ping_buffer;
  incoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

  // Buffer for incoming pong frame_id
  char incoming_pong_buffer[STRING_BUFFER_LEN];
  incoming_pong.frame_id.data = incoming_pong_buffer;
  incoming_pong.frame_id.capacity = STRING_BUFFER_LEN;

  // Generate unique device ID
  device_id = rand();

  // ===================== Main loop =====================
  // Equivalent to rclcpp::spin()

  while (1) {
    // Process timers and subscriptions
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // Yield CPU (important for FreeRTOS)
    usleep(10000);
  }

  // ===================== Cleanup (never reached) =====================

  RCCHECK(rcl_publisher_fini(&ping_publisher, &node));
  RCCHECK(rcl_publisher_fini(&pong_publisher, &node));
  RCCHECK(rcl_subscription_fini(&ping_subscriber, &node));
  RCCHECK(rcl_subscription_fini(&pong_subscriber, &node));
  RCCHECK(rcl_node_fini(&node));
}
