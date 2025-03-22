#ifndef DIFF_DRIVE_ODOMETRY_H
    #define DIFF_DRIVE_ODOMETRY_H

    #include <math.h>

    typedef struct DiffDriveOdom {
        float odom_update_period;
        float wheel_radius;
        float wheel_separation;

        float left_wheel_vel_mps;
        float right_wheel_vel_mps;

        float linear_x;
        float angular_z;

        float pos_x;
        float pos_y;
        float orientation;

    } DiffDriveOdom;

    void reset_odometry(DiffDriveOdom* odom) {
        odom->linear_x = 0.0;
        odom->angular_z = 0.0;

        odom->pos_x = 0.0;
        odom->pos_y = 0.0;
        odom->orientation = 0.0;
    }

    void configure_diff_drive_odom(DiffDriveOdom* odom, float odom_update_period, float wheel_radius, float wheel_separation) {
        odom->odom_update_period = odom_update_period;
        odom->wheel_radius = wheel_radius;
        odom->wheel_separation = wheel_separation;
        reset_odometry(odom);
    }

    void update_odometry(DiffDriveOdom* odom, float left_wheel_feedback_vel, float right_wheel_feedback_vel) {
        if (odom->odom_update_period <= 0) return;

        // Calculating wheel velocity
        odom->left_wheel_vel_mps = left_wheel_feedback_vel * odom->wheel_radius;
        odom->right_wheel_vel_mps = right_wheel_feedback_vel * odom->wheel_radius;

        // Estimate robot speed and position
        odom->linear_x = (odom->right_wheel_vel_mps + odom->left_wheel_vel_mps) * 0.5;
        odom->angular_z = (odom->right_wheel_vel_mps - odom->left_wheel_vel_mps) / odom->wheel_separation;

        odom->pos_x += odom->linear_x * odom->odom_update_period * cos(odom->orientation);
        odom->pos_y += odom->linear_x * odom->odom_update_period * sin(odom->orientation);
        odom->orientation += odom->angular_z * odom->odom_update_period;
    }

    float get_odom_pos_x(DiffDriveOdom odom) {
        return odom.pos_x;
    }

    float get_odom_pos_y(DiffDriveOdom odom) {
        return odom.pos_y;
    }

    float get_odom_orientation(DiffDriveOdom odom) {
        return odom.orientation;
    }

    float get_odom_vel_lin_x(DiffDriveOdom odom) {
        return odom.linear_x;
    }

    float get_odom_vel_ang_z(DiffDriveOdom odom) {
        return odom.angular_z;
    }

#endif