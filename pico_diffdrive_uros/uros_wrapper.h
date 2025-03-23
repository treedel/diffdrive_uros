#ifndef UROS_WRAPPER_H
    #define UROS_WRAPPER_H

    #include <string.h>
    #include <stdbool.h>

    #include "pico_uart_transports.h"

    #include <rmw_microros/rmw_microros.h>
    #include <rclc/rclc.h>
    #include <rclc/executor.h>

    #define ROBOT_NAME_MAX_SIZE 10
    #define TOPIC_NAME_MAX_SIZE 20

    #define RCL_EXECUTOR_PERIOD_MS 1
    #define MAX_PUBLISHERS 5
    #define MAX_SUBSCRIBERS 5
    #define MAX_TIMERS 10

    typedef struct UrosWrapperPublisher {

        float publish_rate;
        char topic_name[TOPIC_NAME_MAX_SIZE+1];

        rcl_publisher_t publisher;

    } UrosWrapperPublisher;

    typedef struct UrosWrapperSubscriber {

        char topic_name[TOPIC_NAME_MAX_SIZE + 1];

        rcl_subscription_t subscriber;
        void *msg;
        rclc_subscription_callback_t callback;
        rclc_executor_handle_invocation_t invocation_type;

    } UrosWrapperSubscriber;

    typedef struct UrosWrapperCore {

        char robot_name[ROBOT_NAME_MAX_SIZE + 1];
        uint8_t n_publishers;
        uint8_t n_subscribers;
        uint8_t n_timers;

        rcl_allocator_t allocator;
        rcl_node_t node;
        rclc_support_t support;
        rclc_executor_t executor;

        rcl_timer_t timers[MAX_TIMERS];
        UrosWrapperSubscriber subscribers[MAX_SUBSCRIBERS];

    } UrosWrapperCore;

    bool configure_uros_wrapper_core(UrosWrapperCore* uros_wrapper, char* robot_name) {
        strcpy(uros_wrapper->robot_name, robot_name);
        uros_wrapper->n_publishers = 0;
        uros_wrapper->n_subscribers = 0;
        uros_wrapper->n_timers = 0;

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
            return false;
        }

        uros_wrapper->allocator = rcl_get_default_allocator();
        rclc_support_init(&(uros_wrapper->support), 0, NULL, &(uros_wrapper->allocator));
        rclc_node_init_default(&(uros_wrapper->node), uros_wrapper->robot_name, "", &(uros_wrapper->support));

        return true;
    }

    void uros_wrapper_add_timer(UrosWrapperCore* uros_wrapper, rcl_timer_callback_t callback,
        float trigger_period_ms, bool repeat) {

            rclc_timer_init_default2(
                &(uros_wrapper->timers[(uros_wrapper->n_timers)++]),
                &(uros_wrapper->support),
                RCL_MS_TO_NS(trigger_period_ms),
                callback,
                repeat
            );

    }

    void uros_wrapper_add_publisher(UrosWrapperCore* uros_wrapper, UrosWrapperPublisher* uros_publisher,
        char* topic_name, const rosidl_message_type_support_t* type_support,
        rcl_timer_callback_t callback, float publish_rate) {

        uros_publisher->publish_rate = publish_rate;

        strcpy(uros_publisher->topic_name, topic_name);

        rclc_publisher_init_default(
            &(uros_publisher->publisher),
            &(uros_wrapper->node),
            type_support,
            topic_name
        );

        (uros_wrapper->n_publishers)++;

        if (!(uros_publisher->publish_rate > 0)) return;

        uros_wrapper_add_timer(uros_wrapper, callback, (1000.0/uros_publisher->publish_rate), true);

    }

    void uros_wrapper_add_subscriber(UrosWrapperCore *uros_wrapper, const char *topic_name,
        const rosidl_message_type_support_t *type_support, void *msg,
        rclc_subscription_callback_t callback, rclc_executor_handle_invocation_t invocation_type) {
    
        strncpy((uros_wrapper->subscribers)[uros_wrapper->n_subscribers].topic_name, topic_name, TOPIC_NAME_MAX_SIZE);
    
        (uros_wrapper->subscribers)[uros_wrapper->n_subscribers].msg = msg;
        (uros_wrapper->subscribers)[uros_wrapper->n_subscribers].callback = callback;
        (uros_wrapper->subscribers)[uros_wrapper->n_subscribers].invocation_type = invocation_type;
    
        // Initialize subscriber
        rcl_ret_t rc = rclc_subscription_init_default(
            &((uros_wrapper->subscribers)[uros_wrapper->n_subscribers].subscriber),
            &(uros_wrapper->node),
            type_support,
            (uros_wrapper->subscribers)[uros_wrapper->n_subscribers].topic_name);
    
        // Increment counters
        uros_wrapper->n_subscribers++;
    }

    void setup_uros_executor(UrosWrapperCore *uros_wrapper) {
    // Increase the number of executor handles to match subscribers
    uint16_t total_handles = uros_wrapper->n_subscribers + uros_wrapper->n_timers;

    rclc_executor_init(
        &(uros_wrapper->executor),
        &((uros_wrapper->support).context),
        total_handles,
        &(uros_wrapper->allocator));

    for (int i = 0; i < uros_wrapper->n_timers; i++) {
        rclc_executor_add_timer(&(uros_wrapper->executor), &(uros_wrapper->timers[i]));
    }

    for (int i = 0; i < uros_wrapper->n_subscribers; i++) {
        // Add subscription to executor
        rcl_ret_t rc = rclc_executor_add_subscription(
            &(uros_wrapper->executor),
            &((uros_wrapper->subscribers)[i].subscriber),
            (uros_wrapper->subscribers)[i].msg,
            (uros_wrapper->subscribers)[i].callback,
            (uros_wrapper->subscribers)[i].invocation_type
        );
    }
}

    void run_uros_executor(UrosWrapperCore* uros_wrapper) {
        rclc_executor_spin_some(&(uros_wrapper->executor), RCL_MS_TO_NS(RCL_EXECUTOR_PERIOD_MS));
    }

    // Utility function to set timestamps of messages
    void set_timestamp_now(int32_t* now_sec, uint32_t* now_nano_sec) {
        int64_t now = rmw_uros_epoch_nanos();
        if (now > 0) {
            *now_sec = now / 1000000000;
            *now_nano_sec = now % 1000000000;
        }
        
        else {
            // Fallback to zero if timestamp retrieval fails
            *now_sec = 0;
            *now_nano_sec = 0;
        }
    }

#endif