
/**
 * @file mpu6050.h
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

#include "HAL_mpu6050.h"

// Register addresses
#define MPU6050_RA_SMPLRT_DIV    0x19
#define MPU6050_RA_CONFIG        0x1A
#define MPU6050_RA_GYRO_CONFIG   0x1B
#define MPU6050_RA_ACCEL_CONFIG  0x1C
#define MPU6050_RA_ACCEL_XOUT_H  0x3B
#define MPU6050_RA_TEMP_OUT_H    0x41
#define MPU6050_RA_GYRO_XOUT_H   0x43
#define MPU6050_RA_PWR_MGMT_1    0x6B
#define MPU6050_RA_WHO_AM_I      0x75

// Gyroscope full scale ranges
#define MPU6050_GYRO_FS_250      0x00
#define MPU6050_GYRO_FS_500      0x01
#define MPU6050_GYRO_FS_1000     0x02
#define MPU6050_GYRO_FS_2000     0x03

// Accelerometer full scale ranges
#define MPU6050_ACCEL_FS_2       0x00
#define MPU6050_ACCEL_FS_4       0x01
#define MPU6050_ACCEL_FS_8       0x02
#define MPU6050_ACCEL_FS_16      0x03

typedef struct 
{
    uint8_t i2c_addr;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
}mpu6050_data_t;


bool mpu6050_init(uint8_t addr);
bool mpu6050_read_data(mpu6050_data_t* data);



#endif