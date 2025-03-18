#ifndef DIFF_DRIVE_ODOMETRY_H
    #define DIFF_DRIVE_ODOMETRY_H

    #include <math.h>

    typedef struct DiffDriveOdom {
        float loop_period;
        float wheel_radius;
        float wheel_separation;

        float left_wheel_pos;
        float right_wheel_pos;
        float old_left_wheel_pos;
        float old_right_wheel_pos;

        float left_wheel_vel;
        float right_wheel_vel;

        float linear_x;
        float angular_z;

        float pos_x;
        float pos_y;
        float orientation;

    } DiffDriveOdom;

    void reset_odometry(DiffDriveOdom* odom) {
        odom->left_wheel_pos = 0.0;
        odom->right_wheel_pos = 0.0;
        odom->old_left_wheel_pos = 0.0;
        odom->old_right_wheel_pos = 0.0;

        odom->linear_x = 0.0;
        odom->angular_z = 0.0;

        odom->pos_x = 0.0;
        odom->pos_y = 0.0;
        odom->orientation = 0.0;
    }

    void configure_diff_drive_odom(DiffDriveOdom* odom, float loop_period, float wheel_radius, float wheel_separation) {
        odom->loop_period = loop_period;
        odom->wheel_radius = wheel_radius;
        odom->wheel_separation = wheel_separation;
        reset_odometry(odom);
    }

    void update_odometry(DiffDriveOdom* odom, float left_wheel_pos, float right_wheel_pos) {
        if (odom->loop_period <= 0) return;

        // Calculating wheel velocity
        odom->left_wheel_pos = left_wheel_pos * odom->wheel_radius;
        odom->right_wheel_pos = right_wheel_pos * odom->wheel_radius;

        odom->left_wheel_vel = (odom->left_wheel_pos - odom->old_left_wheel_pos) / odom->loop_period;
        odom->right_wheel_vel = (odom->right_wheel_pos - odom->old_right_wheel_pos) / odom->loop_period;

        odom->old_left_wheel_pos = odom->left_wheel_pos;
        odom->old_right_wheel_pos = odom->right_wheel_pos;

        // Estimate robot speed and position
        odom->linear_x = (odom->right_wheel_vel + odom->left_wheel_vel) * 0.5;
        odom->angular_z = (odom->right_wheel_vel - odom->left_wheel_vel) / odom->wheel_separation;

        odom->pos_x += odom->linear_x * odom->loop_period * cos(odom->orientation);
        odom->pos_y += odom->linear_x * odom->loop_period * sin(odom->orientation);
        odom->orientation += odom->angular_z * odom->loop_period;
    }

    float get_pos_x(DiffDriveOdom* odom) {
        return odom->pos_x;
    }

    float get_pos_y(DiffDriveOdom* odom) {
        return odom->pos_y;
    }

    float get_orientation(DiffDriveOdom* odom) {
        return odom->orientation;
    }

    float get_vel_lin_x(DiffDriveOdom* odom) {
        return odom->linear_x;
    }

    float get_vel_ang_z(DiffDriveOdom* odom) {
        return odom->angular_z;
    }

#endif