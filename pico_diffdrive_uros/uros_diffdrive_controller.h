#ifndef TWO_PI
    #define TWO_PI 6.28318530718f
#endif

#ifndef UROS_DIFFDRIVE_CONTROLLER_H
    #define UROS_DIFFDRIVE_CONTROLLER_H

    #include <math.h>
    #include <stdbool.h>

    #include "diff_drive_odometry.h"
    #include "uros_wrapper.h"
    #include "uros_jsp.h"

    #include "nav_msgs/msg/odometry.h"
    #include "geometry_msgs/msg/twist.h"
    #include <tf2_msgs/msg/tf_message.h>
    #include <geometry_msgs/msg/transform_stamped.h>

    #define LEFT_WHEEL_JOINT_FRAME_ID "wheel_left_joint"
    #define RIGHT_WHEEL_JOINT_FRAME_ID "wheel_right_joint"
    #define ODOM_HEADER_FRAME_ID "odom"
    #define ODOM_CHILD_FRAME_ID "base_footprint"

    struct DiffDriveController {

        float wheel_radius;
        float wheel_separation;

        float update_frequency;

        float max_linear_x;
        float min_linear_x;
        float max_angular_z;
        float min_angular_z;

        UrosWrapperPublisher odom_publisher;
        nav_msgs__msg__Odometry odom_msg;

        UrosWrapperPublisher tf_publisher;
        tf2_msgs__msg__TFMessage tf_msg;

        UrosWrapperSubscriber cmd_vel_subscriber;
        geometry_msgs__msg__Twist cmd_vel_msg;

        int joint_indices[2];

        float left_wheel_pos_rad;
        float right_wheel_pos_rad;

        float left_wheel_vel;
        float right_wheel_vel;

    } diff_drive_controller;

    DiffDriveOdom odom;

    void set_diff_drive_controller_limits(float max_linear_x, float min_linear_x, float max_angular_z, float min_angular_z) {

        diff_drive_controller.max_linear_x = max_linear_x;
        diff_drive_controller.min_linear_x = min_linear_x;
        diff_drive_controller.max_angular_z = max_angular_z;
        diff_drive_controller.min_angular_z = min_angular_z;
        
    }

    void update_diff_drive_odom_values(float left_wheel_feedback_vel, float right_wheel_feedback_vel) {

        update_odometry(&odom, left_wheel_feedback_vel, right_wheel_feedback_vel);

        // Prepare odom message
        diff_drive_controller.odom_msg.pose.pose.position.x = get_odom_pos_x(odom);
        diff_drive_controller.odom_msg.pose.pose.position.y = get_odom_pos_y(odom);

        // Convert yaw from rads to quaternion
        float yaw_angle = get_odom_orientation(odom);
        diff_drive_controller.odom_msg.pose.pose.orientation.z = sin(yaw_angle / 2.0);
        diff_drive_controller.odom_msg.pose.pose.orientation.w = cos(yaw_angle / 2.0);

        diff_drive_controller.odom_msg.twist.twist.linear.x = get_odom_vel_lin_x(odom);
        diff_drive_controller.odom_msg.twist.twist.angular.z = get_odom_vel_ang_z(odom);

        // Prepare tf message
        diff_drive_controller.tf_msg.transforms.size = 1;
        diff_drive_controller.tf_msg.transforms.data[0].header.stamp = diff_drive_controller.odom_msg.header.stamp;
        diff_drive_controller.tf_msg.transforms.data[0].header.frame_id.data = "odom";
        diff_drive_controller.tf_msg.transforms.data[0].child_frame_id.data = "base_footprint";

        diff_drive_controller.tf_msg.transforms.data[0].transform.translation.x = diff_drive_controller.odom_msg.pose.pose.position.x;
        diff_drive_controller.tf_msg.transforms.data[0].transform.translation.y = diff_drive_controller.odom_msg.pose.pose.position.y;
        diff_drive_controller.tf_msg.transforms.data[0].transform.translation.z = 0.0;

        diff_drive_controller.tf_msg.transforms.data[0].transform.rotation = diff_drive_controller.odom_msg.pose.pose.orientation;

        // Prepare joint states message
        diff_drive_controller.left_wheel_pos_rad += left_wheel_feedback_vel / diff_drive_controller.update_frequency;
        diff_drive_controller.right_wheel_pos_rad += right_wheel_feedback_vel / diff_drive_controller.update_frequency;

        update_joint_position(diff_drive_controller.joint_indices[0], diff_drive_controller.left_wheel_pos_rad);
        update_joint_position(diff_drive_controller.joint_indices[1], diff_drive_controller.right_wheel_pos_rad);

    }
    
    void tf_callback(rcl_timer_t* timer, int64_t last_call_time) {
        
        // Stamp and publish TF message
        set_timestamp_now(
            &diff_drive_controller.tf_msg.transforms.data[0].header.stamp.sec,
            &diff_drive_controller.tf_msg.transforms.data[0].header.stamp.nanosec
        );
        rcl_ret_t ret = rcl_publish(&diff_drive_controller.tf_publisher.publisher, &diff_drive_controller.tf_msg, NULL);

    }

    void odom_callback(rcl_timer_t* timer, int64_t last_call_time) {

        // Stamp and publish odometry message
        set_timestamp_now(
            &diff_drive_controller.odom_msg.header.stamp.sec,
            &diff_drive_controller.odom_msg.header.stamp.nanosec
        );
        rcl_ret_t ret = rcl_publish(&diff_drive_controller.odom_publisher.publisher, &diff_drive_controller.odom_msg, NULL);

        tf_callback(timer, last_call_time);

    }

    void cmd_vel_callback(const void* msg_in) {

        const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*) msg_in;
    
        float linear_x = msg->linear.x;
        float angular_z = msg->angular.z;
        float wheel_radius = diff_drive_controller.wheel_radius;
        float wheel_separation = diff_drive_controller.wheel_separation;
    
        // Limit enforcer
        if (linear_x > 0 && linear_x > diff_drive_controller.max_linear_x) linear_x = diff_drive_controller.max_linear_x;
        else if (linear_x < 0 && linear_x < diff_drive_controller.min_linear_x) linear_x = diff_drive_controller.min_linear_x;

        if (angular_z > 0 && angular_z > diff_drive_controller.max_angular_z) angular_z = diff_drive_controller.max_angular_z;
        else if (angular_z < 0 && angular_z < diff_drive_controller.min_angular_z) angular_z =  diff_drive_controller.min_angular_z;

        diff_drive_controller.left_wheel_vel = (linear_x - (angular_z * wheel_separation / 2.0)) / wheel_radius;
        diff_drive_controller.right_wheel_vel = (linear_x + (angular_z * wheel_separation / 2.0)) / wheel_radius;
        
    }

    float get_diff_drive_left_wheel_vel() {
        return diff_drive_controller.left_wheel_vel;   
    }

    float get_diff_drive_right_wheel_vel() {
        return diff_drive_controller.right_wheel_vel;   
    }

    void configure_diff_drive_controller(UrosWrapperCore* uros_core, float update_frequency,
        float wheel_radius, float wheel_separation, float max_linear_x, float min_linear_x,
        float max_angular_z, float min_angular_z) {

        set_diff_drive_controller_limits(max_linear_x, min_linear_x, max_angular_z, min_angular_z);

        diff_drive_controller.wheel_radius = wheel_radius;
        diff_drive_controller.wheel_separation = wheel_separation;
        diff_drive_controller.update_frequency = update_frequency;

        // odom publisher
        uros_wrapper_add_publisher(
            uros_core,
            &(diff_drive_controller.odom_publisher),
            "/odom",
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            odom_callback,
            diff_drive_controller.update_frequency
        );

        diff_drive_controller.odom_msg.header.frame_id.data = ODOM_HEADER_FRAME_ID;
        diff_drive_controller.odom_msg.child_frame_id.data = ODOM_CHILD_FRAME_ID;

        // tf publisher (Disabled auto publishing)
        uros_wrapper_add_publisher(
            uros_core,
            &(diff_drive_controller.tf_publisher),
            "/tf",
            ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
            tf_callback,
            0
        );

        diff_drive_controller.tf_msg.transforms.data = malloc(sizeof(geometry_msgs__msg__TransformStamped));
        diff_drive_controller.tf_msg.transforms.capacity = 1;
        diff_drive_controller.tf_msg.transforms.size = 1;

        diff_drive_controller.joint_indices[0] = jsp_add_joint(LEFT_WHEEL_JOINT_FRAME_ID);
        diff_drive_controller.joint_indices[1] = jsp_add_joint(RIGHT_WHEEL_JOINT_FRAME_ID);
        configure_jsp(uros_core, diff_drive_controller.update_frequency);

        // cmd_vel subscriber
        uros_wrapper_add_subscriber(
            uros_core,
            "cmd_vel",
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            &(diff_drive_controller.cmd_vel_msg),
            cmd_vel_callback,
            ON_NEW_DATA
        );

        configure_diff_drive_odom(
            &odom,
            (1.0/diff_drive_controller.update_frequency),
            diff_drive_controller.wheel_radius,
            diff_drive_controller.wheel_separation
        );

    }

#endif