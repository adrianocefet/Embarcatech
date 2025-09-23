#ifndef MPU6050_I2C_H
#define MPU6050_I2C_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>

#define MPU6050_ADDR 0x68
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

typedef struct {
    float ax, ay, az;   // aceleração em g
    float gx, gy, gz;   // giros em deg/s
    float angle_x, angle_y, angle_z; // ângulos no espaço cartesiano
} mpu6050_data_t;

bool mpu6050_init(i2c_inst_t *i2c, uint sda, uint scl);
bool mpu6050_read(mpu6050_data_t *data);
void mpu6050_calc_angles(mpu6050_data_t *data);

#endif
