#ifndef UROS_IMU_PUBLISHER_H
    #define UROS_IMU_PUBLISHER_H

    #include "mpu6050.h"
    #include "uros_wrapper.h"

    #include "sensor_msgs/msg/imu.h"

    #define IMU_PUBLISH_RATE 20.0

    struct ImuPublisher {
        
        ImuController imu_controller;

        UrosWrapperPublisher imu_msg_publisher;
        sensor_msgs__msg__Imu imu_msg;
        
    } imu_publisher;


    void imu_callback(rcl_timer_t* timer, int64_t last_call_time) {

        update_mpu6050_readings(&imu_publisher.imu_controller);

        imu_publisher.imu_msg.linear_acceleration.x = get_accel_x(imu_publisher.imu_controller);
        imu_publisher.imu_msg.linear_acceleration.y = get_accel_y(imu_publisher.imu_controller);
        imu_publisher.imu_msg.linear_acceleration.z = get_accel_z(imu_publisher.imu_controller);
        imu_publisher.imu_msg.angular_velocity.x = get_gyro_x(imu_publisher.imu_controller);
        imu_publisher.imu_msg.angular_velocity.y = get_gyro_y(imu_publisher.imu_controller);
        imu_publisher.imu_msg.angular_velocity.z = get_gyro_z(imu_publisher.imu_controller);

        set_timestamp_now(
            &imu_publisher.imu_msg.header.stamp.sec,
            &imu_publisher.imu_msg.header.stamp.nanosec
        );

        rcl_ret_t ret = rcl_publish(&imu_publisher.imu_msg_publisher.publisher, &imu_publisher.imu_msg, NULL);

    }

    void configure_imu_publisher(UrosWrapperCore* uros_core, float accel_offset_x,
        float accel_offset_y, float accel_offset_z, float gyro_offset_x, float gyro_offset_y, float gyro_offset_z) {

        configure_mpu6050(
            &imu_publisher.imu_controller,
            accel_offset_x,
            accel_offset_y,
            accel_offset_z,
            gyro_offset_x,
            gyro_offset_y,
            gyro_offset_z
        );

        uros_wrapper_add_publisher(
            uros_core,
            &imu_publisher.imu_msg_publisher,
            "/imu_data_raw",
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            imu_callback,
            IMU_PUBLISH_RATE
        );

    }

#endif