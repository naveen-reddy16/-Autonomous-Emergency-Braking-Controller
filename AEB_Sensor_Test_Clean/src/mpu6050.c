
#include "mpu6050.h"

#include <devctl.h>
#include <errno.h>
#include <fcntl.h>
#include <hw/i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int i2c_fd = -1;

static int mpu_write_register(uint8_t reg, uint8_t value)
{
    struct {
        i2c_send_t header;
        uint8_t bytes[2];
    } msg;

    memset(&msg, 0, sizeof(msg));

    msg.header.slave.addr = MPU6050_I2C_ADDRESS;
    msg.header.slave.fmt = I2C_ADDRFMT_7BIT;
    msg.header.len = 2;
    msg.header.stop = 1;

    msg.bytes[0] = reg;
    msg.bytes[1] = value;

    int status = devctl(i2c_fd, DCMD_I2C_SEND,
                        &msg, sizeof(msg), NULL);

    if (status != EOK) {
        errno = status;
        perror("MPU6050 register write");
        return -1;
    }

    return 0;
}

static int mpu_read_registers(uint8_t reg,
                              uint8_t *buffer,
                              uint32_t length)
{
    if (i2c_fd < 0 || buffer == NULL || length == 0) {
        return -1;
    }

    size_t allocation_size =
        sizeof(i2c_sendrecv_t) +
        ((length > 1U) ? length : 1U);

    struct i2c_read_message {
        i2c_sendrecv_t header;
        uint8_t bytes[];
    };

    struct i2c_read_message *msg =
        malloc(allocation_size);

    if (msg == NULL) {
        perror("malloc I2C message");
        return -1;
    }

    memset(msg, 0, allocation_size);

    msg->header.slave.addr = MPU6050_I2C_ADDRESS;
    msg->header.slave.fmt = I2C_ADDRFMT_7BIT;
    msg->header.send_len = 1;
    msg->header.recv_len = length;
    msg->header.stop = 1;

    msg->bytes[0] = reg;

    int status = devctl(i2c_fd, DCMD_I2C_SENDRECV,
                        msg, allocation_size, NULL);

    if (status != EOK) {
        errno = status;
        perror("MPU6050 register read");
        free(msg);
        return -1;
    }

    memcpy(buffer, msg->bytes, length);
    free(msg);

    return 0;
}

int mpu6050_init(const char *device_path)
{
    if (device_path == NULL) {
        return -1;
    }

    i2c_fd = open(device_path, O_RDWR);

    if (i2c_fd < 0) {
        perror("open I2C device");
        return -1;
    }

    /* Wake the MPU6050 and select the internal clock. */
    if (mpu_write_register(0x6B, 0x01) != 0 ||
        mpu_write_register(0x1A, 0x03) != 0 ||
        mpu_write_register(0x19, 0x09) != 0 ||
        mpu_write_register(0x1B, 0x00) != 0 ||
        mpu_write_register(0x1C, 0x00) != 0) {
        mpu6050_close();
        return -1;
    }

    return 0;
}

int mpu6050_read(mpu6050_data_t *data)
{
    if (data == NULL) {
        return -1;
    }

    uint8_t raw[14];

    if (mpu_read_registers(0x3B, raw, sizeof(raw)) != 0) {
        return -1;
    }

    data->accel_x = (int16_t)((raw[0] << 8) | raw[1]);
    data->accel_y = (int16_t)((raw[2] << 8) | raw[3]);
    data->accel_z = (int16_t)((raw[4] << 8) | raw[5]);
    data->temperature =
        (int16_t)((raw[6] << 8) | raw[7]);
    data->gyro_x = (int16_t)((raw[8] << 8) | raw[9]);
    data->gyro_y = (int16_t)((raw[10] << 8) | raw[11]);
    data->gyro_z = (int16_t)((raw[12] << 8) | raw[13]);

    return 0;
}

void mpu6050_close(void)
{
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}
