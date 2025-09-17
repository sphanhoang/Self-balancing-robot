/**
 * @file HAL_mpu6050.c
 * @brief MPU6050 HAL for ESP32
 * @author Phan Hoang Son
 * @date 13 Sep 2025
 */

#include "HAL_mpu6050.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


#define I2C_MASTER_SCL_IO           22          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           21          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */


i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

static const char *TAG = "MPU6050_INTERFACE";

uint8_t mpu6050_i2c_init(void)
{
    esp_err_t ret;
    i2c_master_bus_config_t bus_config = 
    {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ret =  i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "i2c master bus init failed", esp_err_to_name(ret));
        return 1;
    }
    i2c_device_config_t dev_config = 
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR_AD0_LOW,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "i2c master bus add device failed", esp_err_to_name(ret));
        return 1;
    }
    ESP_LOGI(TAG, "iic init success");
    return 0;
}

uint8_t mpu6050_reg_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    esp_err_t ret;
    ret = i2c_master_transmit_receive(
        dev_handle,         /* device handle */
        &reg,               /* write: register address*/
        1,                  /* write: 1 byte*/
        buf,                /* read: data*/
        len,                /* read: number of bytes*/
        -1                  /* Wait forever */
    );
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "i2c master transmit receive failed", esp_err_to_name(ret));
        return 1;
    }
    return 0;
}

uint8_t mpu6050_reg_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    esp_err_t ret;
    uint8_t write_buf[len + 1];         /* +1 for register address */
    write_buf[0] = reg;                 /* first byte is register address */
    for (uint16_t i = 0; i < len; i++)  /* Copy data to write_buf */
    {
        write_buf[i + 1] = buf[i];
    }
    ret = i2c_master_transmit
    (
        dev_handle,  /* device handle */
        write_buf,   /* write: register address + data */ 
        len + 1,     /* write: number of bytes */
        -1           /* Wait forever */
    );
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "i2c master transmit failed", esp_err_to_name(ret));
        return 1;
    }    
    return 0;
}

void mpu6050_interface_delay_ms(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

HAL_mpu6050_t HAL_mpu6050 =
{
    .init = mpu6050_i2c_init,
    .i2c_read = mpu6050_reg_read,
    .i2c_write = mpu6050_reg_write,
    .delay_ms = vTaskDelay
};