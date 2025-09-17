/**
 * @file HAL_mpu6050.h
 */
#include <stdint.h>
#include <stdbool.h>

#ifndef HAL_MPU6050_H
#define HAL_MPU6050_H

#define MPU6050_ADDR_AD0_LOW     0x68       /*< MPU6050 IIC 7-bit address >*/
#define MPU6050_ADDR_AD0_HIGH    0x69       /*< MPU6050 IIC 7-bit address >*/

typedef struct 
{
    uint8_t (*init)(void);
    uint8_t (*i2c_read)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
    uint8_t (*i2c_write)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
    void (*delay_ms)(uint32_t ms);
}HAL_mpu6050_t;

#endif