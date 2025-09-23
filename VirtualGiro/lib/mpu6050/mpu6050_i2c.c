#include "mpu6050_i2c.h"
#include <math.h>
#include <string.h>

static i2c_inst_t *i2c_bus = NULL;
static uint8_t mpu_addr = MPU6050_ADDR;

// Write register
static bool mpu6050_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return (i2c_write_blocking(i2c_bus, mpu_addr, buf, 2, false) == 2);
}

// Read bytes
static bool mpu6050_read_bytes(uint8_t reg, uint8_t *buf, uint len) {
    if (i2c_write_blocking(i2c_bus, mpu_addr, &reg, 1, true) != 1) return false;
    return (i2c_read_blocking(i2c_bus, mpu_addr, buf, len, false) == (int)len);
}

// Init MPU6050
bool mpu6050_init(i2c_inst_t *i2c, uint sda, uint scl) {
    i2c_bus = i2c;
    sleep_ms(100);
    return mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00); // wake up
}

// Read raw data
bool mpu6050_read(mpu6050_data_t *data) {
    uint8_t buf[14];
    if (!mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, buf, 14)) return false;

    int16_t raw_ax = (buf[0] << 8) | buf[1];
    int16_t raw_ay = (buf[2] << 8) | buf[3];
    int16_t raw_az = (buf[4] << 8) | buf[5];
    int16_t raw_gx = (buf[8] << 8) | buf[9];
    int16_t raw_gy = (buf[10] << 8) | buf[11];
    int16_t raw_gz = (buf[12] << 8) | buf[13];

    data->ax = raw_ax / 16384.0f;
    data->ay = raw_ay / 16384.0f;
    data->az = raw_az / 16384.0f;

    data->gx = raw_gx / 131.0f;
    data->gy = raw_gy / 131.0f;
    data->gz = raw_gz / 131.0f;

    return true;
}

// Calculate angles X/Y/Z
void mpu6050_calc_angles(mpu6050_data_t *data) {
    float norm = sqrtf(data->ax*data->ax + data->ay*data->ay + data->az*data->az);
    if (norm == 0) return;
    data->angle_x = acosf(data->ax / norm) * 180.0f / M_PI;
    data->angle_y = acosf(data->ay / norm) * 180.0f / M_PI;
    data->angle_z = acosf(data->az / norm) * 180.0f / M_PI;
}
