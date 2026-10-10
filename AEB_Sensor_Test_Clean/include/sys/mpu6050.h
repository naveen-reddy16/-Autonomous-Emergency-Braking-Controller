
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#define MPU6050_I2C_ADDRESS  0x68

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_data_t;

/* Open the QNX I2C device and initialize the MPU6050. */
int mpu6050_init(const char *device_path);

/* Read the seven raw sensor values. */
int mpu6050_read(mpu6050_data_t *data);

/* Close the I2C device. */
void mpu6050_close(void);

#endif
