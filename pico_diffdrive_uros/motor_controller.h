#ifndef MOTOR_CONTROLLER_H
    #define MOTOR_CONTROLLER_H

    #include "motor.h"
    #include "encoder.h"
    #include "pid.h"

    typedef struct MotorController {

        float lpf_alpha;
        float pid_loop_rate;

        float encoder_counts_per_rev;
        float rads_per_count;
        float counts_per_rad;

        Motor motor;
        EncoderCounter encoder_counter;
        PidControl pid_control;
        
        int32_t target_cps;
        float filtered_motor_count_delta;
        int encoder_count_multiplier;

        float motor_feedback_vel;

    } MotorController;

    void configure_motor_controller(MotorController* motor_controller, float lpf_alpha,
        int motor_pin_a,int motor_pin_b, int encoder_counts_per_rev,
        int encoder_pin_ab, int encoder_count_multiplier,
        float kp, float ki, float kd, float pid_loop_rate) {

        motor_controller->lpf_alpha = lpf_alpha;

        motor_controller->encoder_counts_per_rev = encoder_counts_per_rev;
        motor_controller->rads_per_count = TWO_PI / motor_controller->encoder_counts_per_rev;
        motor_controller->counts_per_rad = motor_controller->encoder_counts_per_rev / TWO_PI;

        motor_controller->encoder_count_multiplier = encoder_count_multiplier;
        motor_controller->pid_loop_rate = pid_loop_rate;

        configure_motor(&motor_controller->motor, motor_pin_a, motor_pin_b);
        configure_encoder_counter(&motor_controller->encoder_counter, encoder_pin_ab);
        configure_pid_control(&motor_controller->pid_control, 1.0 / motor_controller->pid_loop_rate, kp, ki, kd);

        set_pid_target(&motor_controller->pid_control, 0);

    }

    void run_closed_loop_control(MotorController* motor_controller) {

        set_pid_target(&motor_controller->pid_control, motor_controller->target_cps);
        update_encoder_values(&motor_controller->encoder_counter);

        int32_t encoder_delta = (motor_controller->encoder_count_multiplier) *
            get_encoder_delta(motor_controller->encoder_counter) *
            motor_controller->pid_loop_rate;

        motor_controller->filtered_motor_count_delta = (motor_controller->lpf_alpha * encoder_delta) +
            ((1 - motor_controller->lpf_alpha) * motor_controller->filtered_motor_count_delta);

        int32_t control_level = calculate_pid_output(
            &motor_controller->pid_control,
            motor_controller->filtered_motor_count_delta
        );

        motor_controller->motor_feedback_vel = encoder_delta * motor_controller->rads_per_count;

        control_motor(&motor_controller->motor, control_level);

        printf("Main: %d (Filtered: %.2f) : %d\n",
            encoder_delta, motor_controller->filtered_motor_count_delta, control_level);

    }

    void set_motor_control_target(MotorController* motor_controller, float target_vel) {
        
        motor_controller->target_cps = target_vel * motor_controller->counts_per_rad;
        run_closed_loop_control(motor_controller);

    }

    float get_motor_speed(MotorController motor_controller) {
        return motor_controller.motor_feedback_vel;
    }

#endif