/**
* Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef MPU6050_H
    #define MPU6050_H

    #include <math.h>

    #include <stdio.h>
    #include <string.h>
    #include "pico/stdlib.h"
    #include "hardware/i2c.h"

    #define MPU6050_ADDR 0x68

    #define ACCEL_SCALE 16384.0f // For 2g full-scale range
    #define GYRO_SCALE 131.0f // For 250 dps full-scale range
    #define DEG_TO_RAD (3.14159265359f / 180.0f)

    typedef struct {

        float accel_offset_x;
        float accel_offset_y;
        float accel_offset_z;

        float gyro_offset_x;
        float gyro_offset_y;
        float gyro_offset_z;

        float accel_x;
        float accel_y;
        float accel_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;

    } ImuController;

    void mpu6050_reset() {

        uint8_t buf[] = {0x6B, 0x80};
        i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);
        sleep_ms(100);
        buf[1] = 0x00;
        i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);
        sleep_ms(10);

    }

    void configure_mpu6050(ImuController* imu_controller, float accel_offset_x, float accel_offset_y,
        float accel_offset_z, float gyro_offset_x, float gyro_offset_y, float gyro_offset_z) {

        imu_controller->accel_offset_x = accel_offset_x;
        imu_controller->accel_offset_y = accel_offset_y;
        imu_controller->accel_offset_z = accel_offset_z;
        imu_controller->gyro_offset_x = gyro_offset_x;
        imu_controller->gyro_offset_y = gyro_offset_y;
        imu_controller->gyro_offset_z = gyro_offset_z;

        i2c_init(i2c_default, 400 * 1000);
        gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
        gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
        
        mpu6050_reset();

        uint8_t buf[2];

        // Set accelerometer to 2g for high sensitivity
        buf[0] = 0x1C; // ACCEL_CONFIG register
        buf[1] = 0x00; // Set to 2g
        i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);

        // Set gyroscope to 250°/s for high sensitivity
        buf[0] = 0x1B; // GYRO_CONFIG register
        buf[1] = 0x00; // Set to 250°/s
        i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);

        // Set Digital Low-Pass Filter (DLPF) to 5Hz
        buf[0] = 0x1A; // CONFIG register
        buf[1] = 0x06; // 5Hz filter
        i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);

    }

    void update_mpu6050_readings(ImuController* imu_controller) {

        uint8_t buffer[6];

        int16_t raw_accel[3], raw_gyro[3];

        // Start reading acceleration registers from register 0x3B for 6 bytes
        uint8_t val = 0x3B;
        i2c_write_blocking(i2c_default, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
        i2c_read_blocking(i2c_default, MPU6050_ADDR, buffer, 6, false);

        for (int i = 0; i < 3; i++) {
            raw_accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        }

        // Now gyro data from reg 0x43 for 6 bytes
        // The register is auto incrementing on each read
        val = 0x43;
        i2c_write_blocking(i2c_default, MPU6050_ADDR, &val, 1, true);
        i2c_read_blocking(i2c_default, MPU6050_ADDR, buffer, 6, false);

        for (int i = 0; i < 3; i++) {
            raw_gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
        }

        imu_controller->accel_x = raw_accel[0] / ACCEL_SCALE * 9.81f + imu_controller->accel_offset_x;
        imu_controller->accel_y = raw_accel[1] / ACCEL_SCALE * 9.81f + imu_controller->accel_offset_y;
        imu_controller->accel_z = raw_accel[2] / ACCEL_SCALE * 9.81f + imu_controller->accel_offset_z;
        imu_controller->gyro_x = raw_gyro[0] / GYRO_SCALE * DEG_TO_RAD + imu_controller->gyro_offset_x;
        imu_controller->gyro_y = raw_gyro[1] / GYRO_SCALE * DEG_TO_RAD + imu_controller->gyro_offset_y;
        imu_controller->gyro_z = raw_gyro[2] / GYRO_SCALE * DEG_TO_RAD + imu_controller->gyro_offset_z;

    }

    float get_accel_x(ImuController imu_controller) {
        return roundf(imu_controller.accel_x * 100) / 100;
    }
    
    float get_accel_y(ImuController imu_controller) {
        return roundf(imu_controller.accel_y * 100) / 100;
    }
    
    float get_accel_z(ImuController imu_controller) {
        return roundf(imu_controller.accel_z * 100) / 100;
    }
    
    float get_gyro_x(ImuController imu_controller) { 
        return roundf(imu_controller.gyro_x * 100) / 100;
    }
    
    float get_gyro_y(ImuController imu_controller) { 
        return roundf(imu_controller.gyro_y * 100) / 100;
    }
    
    float get_gyro_z(ImuController imu_controller) { 
        return roundf(imu_controller.gyro_z * 100) / 100;
    }

#endif