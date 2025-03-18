#ifndef TWO_PI
    #define TWO_PI 6.28318530718f
#endif

#ifndef DIFF_DRIVE_BASE_H
    #define DIFF_DRIVE_BASE_H

    #include <stdbool.h>

    // Struct for Differential drive kinematics controller
    typedef struct DiffDriveControl {
        // Constants for calculation
        float wheel_radius;
        float wheel_separation;
        int encoder_cpr;

        bool set_twist_x_limits;
        float max_twist_x;
        float min_twist_x;
        bool set_angular_z_limits;
        float max_angular_z;
        float min_angular_z;
        
        // Inputs from cmd_vel
        float twist_x;
        float angular_z;

        // Wheel velocity
        float left_wheel_vel;
        float right_wheel_vel;

        // Wheel counts per second
        int left_encoder_cps;
        int right_encoder_cps;

    } DiffDriveControl;

    void enable_diffdrive_controller_limits(DiffDriveControl* controller, float max_twist_x, float min_twist_x, float max_angular_z, float min_angular_z) {
        controller->set_twist_x_limits = true;
        controller->max_twist_x = max_twist_x;
        controller->min_twist_x = min_twist_x;
        controller->set_angular_z_limits = true;
        controller->max_angular_z = max_angular_z;
        controller->min_angular_z = min_angular_z;
    }

    void disable_diffdrive_controller_limits(DiffDriveControl* controller) {
        controller->set_twist_x_limits = false;
        controller->max_twist_x = 0.0;
        controller->min_twist_x = -0.0;
        controller->set_angular_z_limits = false;
        controller->max_angular_z = 0.0;
        controller->min_angular_z = -0.0;
    }

    void configure_diff_drive_controller(DiffDriveControl* controller, float wheel_radius, float wheel_separation, int encoder_cpr) {
        controller->wheel_radius = wheel_radius;
        controller->wheel_separation = wheel_separation;
        controller->encoder_cpr = encoder_cpr;

        // Velocity limits. Disabling by default
        disable_diffdrive_controller_limits(controller);
        
        controller->twist_x = 0.0;
        controller->angular_z = 0.0;
        
        controller->left_wheel_vel = 0.0;
        controller->right_wheel_vel = 0.0;

        controller->left_encoder_cps = 0;
        controller->right_encoder_cps = 0;
    }

    void calculate_diff_drive_cps(DiffDriveControl* controller, float twist_x, float angular_z) {
        /* // Limit enforcer
        if (twist_x > 0 && twist_x > controller->max_twist_x) twist_x = controller->max_twist_x;
        else if (twist_x < 0 && twist_x < controller->min_twist_x) twist_x = controller->min_twist_x;

        if (angular_z > 0 && angular_z > controller->max_angular_z) angular_z = controller->max_angular_z;
        else if (angular_z < 0 && angular_z < controller->min_angular_z) angular_z =  controller->min_angular_z; */

        controller->twist_x = twist_x;
        controller->angular_z = angular_z;

        controller->left_wheel_vel = controller->twist_x - (controller->angular_z * controller->wheel_separation / 2);
        controller->right_wheel_vel = controller->twist_x + (controller->angular_z * controller->wheel_separation / 2);

        controller->left_encoder_cps = (int) ((controller->left_wheel_vel / (TWO_PI * controller->wheel_radius)) * controller->encoder_cpr);
        controller->right_encoder_cps = (int) ((controller->right_wheel_vel / (TWO_PI * controller->wheel_radius)) * controller->encoder_cpr);
    }

    int get_left_encoder_cps(DiffDriveControl* controller) {
        return controller->left_encoder_cps;
    }

    int get_right_encoder_cps(DiffDriveControl* controller) {
        return controller->right_encoder_cps;
    }

#endif