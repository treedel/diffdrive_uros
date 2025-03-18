#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "defaultPins.h"

#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "servo.h"
#include "diff_drive_base.h"
#include "diff_drive_odometry.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "geometry_msgs/msg/twist.h"
#include "nav_msgs/msg/odometry.h"

#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

// Constants
#define TOTAL_CHANNELS 2
#define WHEEL_RADIUS 0.035
#define WHEEL_SEPARATION 0.52
#define ENCODER_CPR 840

#define VEL_LIM_LIN_X 0.21
#define VEL_LIM_ANG_Z 0.78

const float RADS_PER_ENC_COUNT = TWO_PI / ENCODER_CPR;

const uint LED_PIN = 25;

// uROS setup
rcl_node_t node;

// Add odom publisher
rcl_timer_t timer;
rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;

// Add cmd_vel subscriber
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

// Add tf publisher
rcl_publisher_t tf_pub;
tf2_msgs__msg__TFMessage tf_msg;

// Add joint_states publisher
rcl_publisher_t joint_state_pub;
sensor_msgs__msg__JointState joint_state_msg;

rclc_executor_t executor;

// Defining pins
const uint ENC_PINS[TOTAL_CHANNELS] = {20, 16};
const uint MOTOR_PINS[TOTAL_CHANNELS][TOTAL_MOTOR_PINS] = {
  {3, 2},
  {5, 4}
};
const uint DOOR_SERVO_PIN = 22;

// For PID loop control
const float PID_LOOP_RATE = 20.0;
const float PID_LOOP_S = 1.0 / PID_LOOP_RATE;
const float PID_LOOP_MS = PID_LOOP_S * 1000;

// LPF Smoothing Factor (0 < alpha < 1)
const float LPF_ALPHA = 0.3;

Motor left_motor, right_motor;
EncoderCounter left_encoder, right_encoder;
PidControl left_pid_controller, right_pid_controller;
DiffDriveControl diff_drive_controller;
DiffDriveOdom odom;

// LPF state variables
float left_filtered_speed = 0;
float right_filtered_speed = 0;

void publish_joint_states() {
  int64_t now = rmw_uros_epoch_nanos();
  if (now > 0) {
      joint_state_msg.header.stamp.sec = now / 1000000000;
      joint_state_msg.header.stamp.nanosec = now % 1000000000;
  } else {
      joint_state_msg.header.stamp.sec = 0;
      joint_state_msg.header.stamp.nanosec = 0;
  }

  // Update wheel positions (radians)
  joint_state_msg.position.data[0] = -get_encoder_counts(left_encoder) * RADS_PER_ENC_COUNT;
  joint_state_msg.position.data[1] = get_encoder_counts(right_encoder) * RADS_PER_ENC_COUNT;

  rcl_ret_t ret = rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
}

void publish_odometry() {
  // Populate and publish odometry message
  int64_t now = rmw_uros_epoch_nanos();
  if (now > 0) {
      odom_msg.header.stamp.sec = now / 1000000000;  // Convert nanoseconds to seconds
      odom_msg.header.stamp.nanosec = now % 1000000000;  // Get remaining nanoseconds
  } else {
      // Fallback to zero if timestamp retrieval fails
      odom_msg.header.stamp.sec = 0;
      odom_msg.header.stamp.nanosec = 0;
  }

  odom_msg.pose.pose.position.x = get_pos_x(&odom);
  odom_msg.pose.pose.position.y = get_pos_y(&odom);

  odom_msg.twist.twist.linear.x = get_vel_lin_x(&odom);
  odom_msg.twist.twist.angular.z = get_vel_ang_z(&odom);

  // Convert yaw from rads to quaternion
  float yaw_angle = get_orientation(&odom);
  odom_msg.pose.pose.orientation.z = sin(yaw_angle / 2.0);
  odom_msg.pose.pose.orientation.w = cos(yaw_angle / 2.0);
  rcl_ret_t ret = rcl_publish(&odom_pub, &odom_msg, NULL);

  // Populate TF Message
  tf_msg.transforms.size = 1;
  tf_msg.transforms.data[0].header.stamp = odom_msg.header.stamp;
  tf_msg.transforms.data[0].header.frame_id.data = "odom";
  tf_msg.transforms.data[0].child_frame_id.data = "base_footprint";

  tf_msg.transforms.data[0].transform.translation.x = odom_msg.pose.pose.position.x;
  tf_msg.transforms.data[0].transform.translation.y = odom_msg.pose.pose.position.y;
  tf_msg.transforms.data[0].transform.translation.z = 0.0;

  tf_msg.transforms.data[0].transform.rotation = odom_msg.pose.pose.orientation;

  // Publish TF transform
  ret = rcl_publish(&tf_pub, &tf_msg, NULL);
}

void pid_control_run(rcl_timer_t *timer, int64_t last_call_time) {
  update_encoder_values(&left_encoder);
  update_encoder_values(&right_encoder);

  // One motor is oriented in reverse
  int32_t left_encoder_delta = -get_encoder_delta(left_encoder) * PID_LOOP_RATE;
  int32_t right_encoder_delta = get_encoder_delta(right_encoder) * PID_LOOP_RATE;

  // Apply Low-Pass Filter (LPF)
  left_filtered_speed = (LPF_ALPHA * left_encoder_delta) + ((1 - LPF_ALPHA) * left_filtered_speed);
  right_filtered_speed = (LPF_ALPHA * right_encoder_delta) + ((1 - LPF_ALPHA) * right_filtered_speed);

  // Use filtered values in PID calculation
  int32_t left_control_level = calculate_pid_output(&left_pid_controller, left_filtered_speed);
  int32_t right_control_level = calculate_pid_output(&right_pid_controller, right_filtered_speed);

  // Use the PID output to drive the motors
  control_motor(&left_motor, left_control_level);
  control_motor(&right_motor, right_control_level);

  // Update odometry
  update_odometry(
    &odom,
    -get_encoder_counts(left_encoder) * RADS_PER_ENC_COUNT,
    get_encoder_counts(right_encoder) * RADS_PER_ENC_COUNT
  );
  
  publish_odometry();
  publish_joint_states();

  /* printf("Left: %d (Filtered: %.2f) : %d, Right %d (Filtered: %.2f) : %d\n",
         left_encoder_delta, left_filtered_speed, left_control_level,
         right_encoder_delta, right_filtered_speed, right_control_level); */
}

void cmd_vel_callback(const void * msg_in) {
    const geometry_msgs__msg__Twist *msg = (geometry_msgs__msg__Twist*) msg_in;
    calculate_diff_drive_cps(&diff_drive_controller, msg->linear.x, msg->angular.z);
    set_pid_target(&left_pid_controller, get_left_encoder_cps(&diff_drive_controller));
    set_pid_target(&right_pid_controller, get_right_encoder_cps(&diff_drive_controller));
}

int main() {
    stdio_init_all();

    // Allow time for serial monitor to start
    sleep_ms(2000);

    // Configure servo
    configure_servo(DOOR_SERVO_PIN);

    // Configure encoders
    configure_encoder_counter(&left_encoder, ENC_PINS[0]);
    configure_encoder_counter(&right_encoder, ENC_PINS[1]);
    
    // Configure motors
    configure_motor(&left_motor, MOTOR_PINS[0][0], MOTOR_PINS[0][1]);
    configure_motor(&right_motor, MOTOR_PINS[1][0], MOTOR_PINS[1][1]);

    // Configure PID controllers
    configure_pid_control(&left_pid_controller, PID_LOOP_S, 10, 100, 0);
    configure_pid_control(&right_pid_controller, PID_LOOP_S, 10, 100, 0);

    // Configure differential drive controller
    configure_diff_drive_controller(&diff_drive_controller, WHEEL_RADIUS, WHEEL_SEPARATION, ENCODER_CPR);
    enable_diffdrive_controller_limits(&diff_drive_controller, VEL_LIM_LIN_X, -VEL_LIM_LIN_X, VEL_LIM_ANG_Z, -VEL_LIM_ANG_Z);

    // Configure differential drive odom handler
    configure_diff_drive_odom(&odom, PID_LOOP_S, WHEEL_RADIUS, WHEEL_SEPARATION);
    
    // Halt the motors initially
    set_pid_target(&left_pid_controller, 0);
    set_pid_target(&right_pid_controller, 0);

    // Configure uROS
    rmw_uros_set_custom_transport(
      true,
      NULL,
      pico_serial_transport_open,
      pico_serial_transport_close,
      pico_serial_transport_write,
      pico_serial_transport_read
    );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) {
        return -1;
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "diff_drive_node", "", &support);

    rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");

    rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
    rclc_publisher_init_default(&tf_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf");
    rclc_publisher_init_default(&joint_state_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states");

    rclc_timer_init_default2(
      &timer,
      &support,
      RCL_MS_TO_NS(PID_LOOP_MS),
      pid_control_run,
      true
    );

    gpio_put(LED_PIN, 1);

    rclc_executor_init(&executor, &support.context, 4, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    odom_msg.header.frame_id.data = "odom";
    odom_msg.child_frame_id.data = "base_footprint";

    tf_msg.transforms.data = malloc(sizeof(geometry_msgs__msg__TransformStamped));
    tf_msg.transforms.capacity = 1;
    tf_msg.transforms.size = 1;

    joint_state_msg.name.data = malloc(3 * sizeof(rosidl_runtime_c__String));
    joint_state_msg.name.size = 3;
    joint_state_msg.name.capacity = 3;

    // Initialize the `name` array properly
    rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 2);  // Set size to 2

    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "wheel_left_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "wheel_right_joint");

    rosidl_runtime_c__double__Sequence__init(&joint_state_msg.position, 2);
    joint_state_msg.position.size = 2;
    joint_state_msg.position.capacity = 2;

    while (true) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}
