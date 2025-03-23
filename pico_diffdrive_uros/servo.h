#ifndef SERVO_H
#define SERVO_H

    #include <hardware/pwm.h>
    #include <hardware/gpio.h>
    #include <hardware/clocks.h>

    #define PWM_FREQ 50
    #define WRAP 10000

    void set_servo_angle(uint gpio, float angle) {
        uint slice_num = pwm_gpio_to_slice_num(gpio);
        float duty_cycle = (angle / 180.0f) * 2000.0f + 500.0f;
        pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * WRAP / 20000.0f));
    }
    
    void configure_servo(uint gpio) {
        gpio_set_function(gpio, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(gpio);
        
        pwm_config config = pwm_get_default_config();
        pwm_config_set_wrap(&config, WRAP);
        pwm_config_set_clkdiv(&config, (float) clock_get_hz(clk_sys) / (PWM_FREQ * WRAP));
        
        pwm_init(slice_num, &config, true);
        set_servo_angle(gpio, 90);
    }

    

#endif