#ifndef UROS_SERVO_CONTROLLER_H
    #define UROS_SERVO_CONTROLLER_H

    #include "servo.h"

    #include "uros_wrapper.h"
    #include <std_msgs/msg/float32_multi_array.h>

    #define MAX_SERVOS 5
    #define SERVO_CONTROLLER_TOPIC "/servos"

    struct ServoController {

        UrosWrapperPublisher servo_publisher;
        std_msgs__msg__Float32MultiArray servo_msg;

        uint8_t servo_pins[MAX_SERVOS];
        uint8_t n_servos;
        
    } servo_controller;

    void servo_controller_callback(const void* msg_in) {
        const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*) msg_in;

        if (msg->data.size > 0 && msg->data.data != NULL) {
            for (size_t i = 0; i < msg->data.size && i < servo_controller.n_servos; i++) {
                set_servo_angle(servo_controller.servo_pins[i], msg->data.data[i]);
            }
        }
        
    }

    void servo_controller_attach(uint8_t pin) {

        servo_controller.servo_pins[servo_controller.n_servos] = pin;
        configure_servo(servo_controller.servo_pins[servo_controller.n_servos]);
        servo_controller.n_servos++;

    }

    void configure_servo_controller(UrosWrapperCore* uros_core) {

        servo_controller.servo_msg.data.capacity = MAX_SERVOS;
        servo_controller.servo_msg.data.data = (float*) malloc(MAX_SERVOS * sizeof(float));
        servo_controller.servo_msg.data.size = 0;

        // Subscriber
        uros_wrapper_add_subscriber(
            uros_core,
            SERVO_CONTROLLER_TOPIC,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            &(servo_controller.servo_msg),
            servo_controller_callback,
            ON_NEW_DATA
        );

    }

#endif