#include <stdio.h>
#include "pico/stdlib.h"

#include "motor_controller.h"
#include "uros_diffdrive_controller.h"
#include "uros_imu_publisher.h"
#include "uros_servo_controller.h"

// Constants
#define TOTAL_CHANNELS 2
#define WHEEL_RADIUS 0.035
#define WHEEL_SEPARATION 0.52
#define ENCODER_CPR 840

#define VEL_LIM_LIN_X 0.25
#define VEL_LIM_ANG_Z 0.785

// MPU6050 offsets
#define USE_PRESET_OFFSETS

#ifdef USE_PRESET_OFFSETS
    #define OFFSET_ACCEL_X 0.75
    #define OFFSET_ACCEL_Y 0.61
    #define OFFSET_ACCEL_Z 0.5

    #define OFFSET_GYRO_X 0.0291
    #define OFFSET_GYRO_Y 0.0098
    #define OFFSET_GYRO_Z -0.003
#endif

#ifndef USE_PRESET_OFFSETS
    #define OFFSET_ACCEL_X 0.0
    #define OFFSET_ACCEL_Y 0.0
    #define OFFSET_ACCEL_Z 0.0

    #define OFFSET_GYRO_X 0.0
    #define OFFSET_GYRO_Y 0.0
    #define OFFSET_GYRO_Z 0.0
#endif


MotorController left_motor_controller;
MotorController right_motor_controller;

UrosWrapperCore uros_core;

// Defining pins
const uint8_t SERVO_PIN = 2;
const uint8_t ENC_PINS[TOTAL_CHANNELS] = {20, 16};
const uint8_t MOTOR_PINS[TOTAL_CHANNELS][TOTAL_MOTOR_PINS] = {
  {7, 6},
  {9, 8}
};

// Motor closed loop control system parameters
const float MCS_LOOP_RATE = 20.0;

// LPF Smoothing Factor (0 < alpha < 1)
const float LPF_ALPHA = 0.3;

void run_motor_control_system() {
    
    set_motor_control_target(&left_motor_controller, get_diff_drive_left_wheel_vel());
    set_motor_control_target(&right_motor_controller, get_diff_drive_right_wheel_vel());

    update_diff_drive_odom_values(
        get_motor_speed(left_motor_controller),
        get_motor_speed(right_motor_controller)
    );

}

int main() {

    stdio_init_all();
    sleep_ms(2000);
    
    configure_motor_controller(
        &left_motor_controller,
        LPF_ALPHA,
        MOTOR_PINS[0][0],
        MOTOR_PINS[0][1],
        ENCODER_CPR,
        ENC_PINS[0],
        -1.0,
        1.0, 100.0, 0.0,
        MCS_LOOP_RATE
    );

    configure_motor_controller(
        &right_motor_controller,
        LPF_ALPHA,
        MOTOR_PINS[1][0],
        MOTOR_PINS[1][1],
        ENCODER_CPR,
        ENC_PINS[1],
        1.0,
        1.0, 100.0, 0.0,
        MCS_LOOP_RATE
    );

    configure_uros_wrapper_core(&uros_core, "diffbot");

    uros_wrapper_add_timer(&uros_core, run_motor_control_system, 1000.0 / MCS_LOOP_RATE, true);

    configure_imu_publisher(
        &uros_core,
        OFFSET_ACCEL_X,
        OFFSET_ACCEL_Y,
        OFFSET_ACCEL_Z,
        OFFSET_GYRO_X,
        OFFSET_GYRO_Y,
        OFFSET_GYRO_Z
    );
    
    configure_diff_drive_controller(&uros_core,
        MCS_LOOP_RATE,
        WHEEL_RADIUS, 
        WHEEL_SEPARATION,
        VEL_LIM_LIN_X,
        -VEL_LIM_LIN_X,
        VEL_LIM_ANG_Z,
        -VEL_LIM_ANG_Z
    );

    servo_controller_attach(SERVO_PIN);
    configure_servo_controller(&uros_core);

    setup_uros_executor(&uros_core);

    while (true) {
        run_uros_executor(&uros_core);
    }

}