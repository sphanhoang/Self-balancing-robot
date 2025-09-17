/**
 * Author: Son Phan
 * Date: 2025-09-12
 */
#include "driver/gpio.h"
#include "driver_mpu6050_dmp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"

static const char *TAG = "MPU6050_DMAIN";

uint32_t i;
uint32_t times;
uint32_t cnt;
uint16_t len;
uint8_t (*g_gpio_irq)(void) = NULL;
static int16_t gs_accel_raw[128][3];
static float gs_accel_g[128][3];
static int16_t gs_gyro_raw[128][3];      
static float gs_gyro_dps[128][3];        
static int32_t gs_quat[128][4];          
static float gs_pitch[128];              
static float gs_roll[128];                
static float gs_yaw[128];                     
mpu6050_address_t addr;

static void a_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU6050_INTERRUPT_MOTION :
        {
            mpu6050_interface_debug_print("mpu6050: irq motion.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_FIFO_OVERFLOW :
        {
            mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_I2C_MAST :
        {
            mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DMP :
        {
            mpu6050_interface_debug_print("mpu6050: irq dmp\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DATA_READY :
        {
            mpu6050_interface_debug_print("mpu6050: irq data ready\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");
            
            break;
        }
    }
}

static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
        case MPU6050_DMP_TAP_X_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq x up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_X_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq x down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq y up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq y down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq z up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq z down with %d.\n", count);
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq unknown code.\n");
            
            break;
        }
    }
}

static void a_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
        case MPU6050_DMP_ORIENT_PORTRAIT :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_LANDSCAPE :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq landscape.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_PORTRAIT :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq reverse portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq reverse landscape.\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq unknown code.\n");
            
            break;
        }
    }
}

void app_main() 
{
    uint8_t res;
    uint32_t i, times = 3;
    uint16_t len; // Will hold the number of data packets read
    mpu6050_address_t addr = MPU6050_ADDRESS_AD0_LOW;

    // 1. Initialize the underlying I2C HAL and GPIO for interrupts
    if (mpu6050_interface_iic_init() != 0)
    {
        ESP_LOGE(TAG, "I2C interface init failed.");
        return;
    }
    // Initialize GPIO interrupt for data ready pin
    // if (gpio_interrupt_init() != 0) // Not yet implemented
    // {
    //     ESP_LOGE(TAG, "GPIO interrupt init failed.");
    //     mpu6050_interface_iic_deinit();
    //     return;
    // }
    g_gpio_irq = mpu6050_dmp_irq_handler; // Link libdriver's handler to ISR

    // 2. Initialize the MPU6050 in DMP mode
    // This function does the heavy lifting: loads DMP firmware, sets up FIFO, etc.
    res = mpu6050_dmp_init(addr, a_receive_callback, 
                     a_dmp_tap_callback, a_dmp_orient_callback);
    if (res != 0)
    {
        ESP_LOGE(TAG, "DMP init failed: %d", res);
        vTaskDelay(pdMS_TO_TICKS(1000));
        mpu6050_interface_iic_deinit();
        return;
    }
    ESP_LOGI(TAG, "DMP initialized successfully.");

    // 3. Short delay for stabilization
    mpu6050_interface_delay_ms(500);

    // 4. Main loop: Poll the FIFO for data
    for (i = 0; i < times; i++)
    {
        len = 128; // Start with the size of our buffer

        // This is the key function. It reads all available processed data packets from the FIFO.
        res = mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                   gs_gyro_raw, gs_gyro_dps,
                                   gs_quat,
                                   gs_pitch, gs_roll, gs_yaw,
                                   &len); // 'len' is updated with the number of packets read
        if (res != 0)
        {
            ESP_LOGE(TAG, "DMP read failed: %d", res);
            (void)mpu6050_dmp_deinit();
            g_gpio_irq = NULL;
            // (void)gpio_interrupt_deinit();
            break;
        }

        if (len > 0)
        {
            // Success! Process the latest data packet (the last one in the array)
            uint16_t latest_index = len - 1;

            ESP_LOGI(TAG, "FIFO Packets: %d", len);
            ESP_LOGI(TAG, "Yaw: %.2f°, Pitch: %.2f°, Roll: %.2f°",
                   gs_yaw[latest_index], gs_pitch[latest_index], gs_roll[latest_index]);
            ESP_LOGI(TAG, "Accel: X=%.2fg, Y=%.2fg, Z=%.2fg",
                   gs_accel_g[latest_index][0], gs_accel_g[latest_index][1], gs_accel_g[latest_index][2]);
            // Use the processed angles (pitch/roll) for your balancing algorithm here
        }
        else
        {
            ESP_LOGD(TAG, "No new data in FIFO.");
        }

        // Delay determines your control loop rate. FIFO buffers data in the meantime.
        vTaskDelay(pdMS_TO_TICKS(10)); // e.g., 100Hz loop
    }

    // 5. Deinit and cleanup
    (void)mpu6050_dmp_deinit();
    g_gpio_irq = NULL;
    // (void)gpio_interrupt_deinit();
}