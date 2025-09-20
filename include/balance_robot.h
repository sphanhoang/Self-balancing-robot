#include "MPU6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <freertos/semphr.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

/* Motor Pins assigment */

#define MOTOR_ENA_PIN   14  
#define MOTOR_ENB_PIN   32
#define MOTOR_IN1_PIN   27
#define MOTOR_IN2_PIN   26
#define MOTOR_IN3_PIN   25
#define MOTOR_IN4_PIN   33

/* Motor constrains */
#define MAX_MOTOR_SPEED 255.0f
#define MIN_MOTOR_SPEED 38.25f

/* PID parameter */
#define KP_PITCH 50.0f
#define KP_YAW 50.0f
#define KI_PITCH 0.0f
#define KI_YAW 0.0f
#define KD_PITCH 0.0f
#define KD_YAW 0.0f

/* Task freq */
#define MOTOR_UPDATE_FREQ_HZ 100  /* 100Hz */ 
#define SENSOR_READ_FREQ_HZ 100
#define CONTROL_LOOP_FREQ_HZ 100

// Data Structures
typedef struct 
{
    float yaw;
    float pitch;
    float roll;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    uint32_t timestamp;
} sensor_data_t;

typedef struct 
{
    float setpoint;
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output;
} pid_controller_t;

typedef struct 
{
    float pitch_output;
    float yawRate_output;
    float left_motor_speed;
    float left_motor_feedback;
    float right_motor_speed;
    float right_motor_feedback;
    uint32_t timestamp;
} control_data_t;