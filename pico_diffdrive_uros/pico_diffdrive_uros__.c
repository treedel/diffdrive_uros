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

// Constants
#define TOTAL_CHANNELS 2
#define WHEEL_RADIUS 0.035
#define WHEEL_SEPARATION 0.52
#define ENCODER_CPS 840

#define VEL_LIM_LIN_X 0.21
#define VEL_LIM_ANG_Z 0.78

// uROS setup
rcl_node_t node;
rcl_subscription_t cmd_vel_sub;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_vel_msg;

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

// LPF state variables
float left_filtered_speed = 0;
float right_filtered_speed = 0;

bool pid_control_run(__unused struct repeating_timer *t) {
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

  /* printf("Left: %d (Filtered: %.2f) : %d, Right %d (Filtered: %.2f) : %d\n",
         left_encoder_delta, left_filtered_speed, left_control_level,
         right_encoder_delta, right_filtered_speed, right_control_level); */

  return true;
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
    configure_diff_drive_controller(&diff_drive_controller, WHEEL_RADIUS, WHEEL_SEPARATION, ENCODER_CPS);
    //enable_diffdrive_controller_limits(&diff_drive_controller, VEL_LIM_LIN_X, -VEL_LIM_LIN_X, VEL_LIM_ANG_Z, -VEL_LIM_ANG_Z);
    
    // Halt the motors initially
    set_pid_target(&left_pid_controller, 0);
    set_pid_target(&right_pid_controller, 0);

    struct repeating_timer timer;
    add_repeating_timer_ms(PID_LOOP_MS, pid_control_run, NULL, &timer);

    // Configure uROS
    rmw_uros_set_custom_transport(
      true,
      NULL,
      pico_serial_transport_open,
      pico_serial_transport_close,
      pico_serial_transport_write,
      pico_serial_transport_read
    );

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
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

    while (true) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}
