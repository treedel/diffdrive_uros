#ifndef EASY_PWM_H
    #define EASY_PWM_H

    #include "hardware/pwm.h"
    #include "hardware/gpio.h"

    // Struct for PWM
    typedef struct PwmControl {
        uint pin;
        uint slice_num;
        uint level;
    } PwmControl;

    // Function to configure PWM pins
    void pwm_configure(PwmControl* pwm_control, uint pin) {
        pwm_control->pin = pin;

        // Set PWM function to the pin
        gpio_set_function(pwm_control->pin, GPIO_FUNC_PWM);
        pwm_control->slice_num = pwm_gpio_to_slice_num(pwm_control->pin);

        // Configure PWM
        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv(&config, 4.f);
        pwm_init(pwm_control->slice_num, &config, true);

        pwm_control->level = 0;
    }

    // Function to set PWM duty cycle
    void set_pwm_duty_cycle(PwmControl* pwm_control, uint level) {
        if (level > 65535) level = 65535;
        if (level < 0) level = 0;
        pwm_control->level = level;
        pwm_set_gpio_level(pwm_control->pin, pwm_control->level);
    }

#endif