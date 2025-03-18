#ifndef MOTOR_H
    #define MOTOR_H

    #include "easy_pwm.h"

    // Constants
    #define TOTAL_MOTOR_PINS 2
    #define MIN_MOTOR_PWM 8000

    // Struct for handling motor
    typedef struct Motor {
        PwmControl pwm_a;
        PwmControl pwm_b;
        uint level;
        uint direction; // true - CW ; false - CCW
    } Motor;

    // Configure motor
    void run_motor(Motor* motor) {
        if (motor->direction) {
            set_pwm_duty_cycle(&motor->pwm_a, motor->level);
            set_pwm_duty_cycle(&motor->pwm_b, 0);
            return;   
        }
        set_pwm_duty_cycle(&motor->pwm_a, 0);
        set_pwm_duty_cycle(&motor->pwm_b, motor->level);
    }

    void configure_motor(Motor* motor, uint pin_a, uint pin_b) {
        pwm_configure(&motor->pwm_a, pin_a);
        pwm_configure(&motor->pwm_b, pin_b);
        motor->level = 0;
        motor->direction = true;
        run_motor(motor);
    }

    void set_motor_direction(Motor* motor, bool direction) {
        motor->direction = direction;
    }

    void set_motor_level(Motor* motor, uint level) {
        if (level>0 && level<MIN_MOTOR_PWM) level += MIN_MOTOR_PWM;
        if (level<0 && level>-MIN_MOTOR_PWM) level -= MIN_MOTOR_PWM;
        motor->level = level;
    }

    void control_motor(Motor* motor, int control_value) {
        if (control_value < 0) set_motor_direction(motor, false);
        else set_motor_direction(motor, true);

        control_value = (control_value<0) ? -control_value:control_value;

        set_motor_level(motor, control_value);
        run_motor(motor);
    }

#endif