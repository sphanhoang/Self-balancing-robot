/**
 * @file balancing_robot.h
 * @brief Self-balancing robot control system header
 * @author Son Phan
 * @date Sept 2025
 */

// #include "MPU6050.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include <freertos/semphr.h>
// #include "esp_err.h"
// #include "esp_log.h"
// #include "esp_timer.h"
// #include "driver/gpio.h"
// #include "driver/mcpwm.h"
// #include "soc/mcpwm_periph.h"
// #include "sdkconfig.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include "esp_timer.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "sdkconfig.h"

/* I2C parameters */
#define PIN_SDA                 21
#define PIN_CLK                 22
#define I2C_MASTER_NUM          I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ      100000                     /*!< I2C master clock frequency */

/* Motor pins assigment */
#define MOTOR_ENA_PIN           14  
#define MOTOR_ENB_PIN           32
#define MOTOR_IN1_PIN           27
#define MOTOR_IN2_PIN           26
#define MOTOR_IN3_PIN           25
#define MOTOR_IN4_PIN           33
#define GPIO_MOTOR_PIN_SEL      (   (1ULL << MOTOR_IN1_PIN) | (1ULL << MOTOR_IN2_PIN) | \
                                    (1ULL << MOTOR_IN3_PIN) | (1ULL << MOTOR_IN4_PIN) )

/* Encoder pin assignment */
#define LEFT_ENCODER_A_PIN      16
#define LEFT_ENCODER_B_PIN      17
#define RIGHT_ENCODER_A_PIN     18
#define RIGHT_ENCODER_B_PIN     19
#define GPIO_ENCODER_PIN_SEL    (   (1ULL << LEFT_ENCODER_A_PIN) | (1ULL << LEFT_ENCODER_B_PIN) | \
                                    (1ULL << RIGHT_ENCODER_A_PIN) | (1ULL << RIGHT_ENCODER_A_PIN) )
#define ENCODER_PPR             22                        /* Pulses per revolution, 11 each channel */
#define WHEEL_PPR               (ENCODER_PPR * 21.3f)f    /* Gear ratio 21.3:1 */   
#define ESP_INTR_FLAG_DEFAULT   0

/* Motor constrains */
#define MAX_MOTOR_SPEED         255.0f
#define MIN_MOTOR_SPEED         10.0f           //38.25
#define MAX_ANGLE               15.0f           /* degrees */
#define MIN_ANGLE               -MAX_ANGLE      /* degrees */
#define PWM_FREQ_HZ             100             /* Hz */

/* PID parameter */
#define KP_ROLL                 25.0f /* best results so far: P = 25, I = 2, D = .8*/
#define KI_ROLL                 4.0f
#define KD_ROLL                 0.8f
#define KP_SPEED                0.01f
#define KI_SPEED                0.01f
#define KD_SPEED                0.01f

/* PID constraints */
#define ROLL_PID_INTEGRAL_CLAMP     100.0f
#define ROLL_PID_OUTPUT_CLAMP       255.0f
#define SPEED_PID_INTEGRAL_CLAMP    100.0f
#define SPEED_PID_OUTPUT_CLAMP      255.0f

/* Task freq */
#define MOTOR_UPDATE_FREQ_HZ        100  /* 100Hz */ 
#define SENSOR_READ_FREQ_HZ         100
#define CONTROL_LOOP_FREQ_HZ        100
#define ENCODER_READ_FREQ_HZ        10

// Data Structures
typedef struct 
{
    float yaw;
    float pitch;
    float roll;
    // float gyro_x;
    // float gyro_y;
    // float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    uint32_t timestamp;
    float dt_since_last;
} sensor_data_t;

typedef struct 
{
    float setpoint;
    float error;
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output;
    float integral_clamp;
    float output_clamp;
} pid_controller_t;

typedef struct 
{
    float roll_output;
    float speed_output;
    float left_motor_speed;
    float right_motor_speed;
    float left_motor_feedback;
    float right_motor_feedback;
    uint32_t timestamp;
} control_data_t;

typedef struct 
{
    uint32_t left_encoder_pulseCount;
    uint32_t right_encoder_pulseCount;
    float left_motor_speed;
    float right_motor_speed;
}encoder_data_t;


void initI2C(void);
void mpu_setup(MPU6050 &mpu);
void sensor_task (void *pvParameters);
float compute_pid(pid_controller_t *pid, float *feedback, float dt);
void balance_task(void *pvParameters);
esp_err_t motor_init(void);
void set_motor_speed(int motor, float *speed);
void motor_task (void *pvParameters);
void monitor_task(void *pvParameters);
float compute_motor_speed(uint32_t &count, float &dt);