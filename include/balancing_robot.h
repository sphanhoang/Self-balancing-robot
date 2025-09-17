/**
 * @file balancing_robot.h
 * @brief Self-balancing robot control system header
 * @author Your Name
 * @date 2024
 */

#ifndef BALANCING_ROBOT_H
#define BALANCING_ROBOT_H

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <math.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Hardware Configuration
#define PIN_SDA 21
#define PIN_SCL 22
#define I2C_NUM I2C_NUM_0
#define I2C_FREQ_HZ 400000

// Motor Control Pins (adjust according to your motor driver)
#define MOTOR_LEFT_PWM_PIN 18
#define MOTOR_LEFT_DIR_PIN 19
#define MOTOR_RIGHT_PWM_PIN 16
#define MOTOR_RIGHT_DIR_PIN 17

// Task Priorities (higher number = higher priority)
#define SENSOR_TASK_PRIORITY 5
#define CONTROL_TASK_PRIORITY 4
#define MOTOR_TASK_PRIORITY 3
#define MONITOR_TASK_PRIORITY 2

// Task Stack Sizes
#define SENSOR_TASK_STACK_SIZE 4096
#define CONTROL_TASK_STACK_SIZE 4096
#define MOTOR_TASK_STACK_SIZE 2048
#define MONITOR_TASK_STACK_SIZE 2048

// Control Parameters
#define CONTROL_LOOP_FREQ_HZ 100  // 100Hz control loop
#define SENSOR_READ_FREQ_HZ 200   // 200Hz sensor reading
#define MOTOR_UPDATE_FREQ_HZ 100  // 100Hz motor update

// PID Parameters (tune these for your robot)
#define KP_ANGLE 50.0f
#define KI_ANGLE 0.0f
#define KD_ANGLE 1.0f

#define KP_SPEED 0.5f
#define KI_SPEED 0.0f
#define KD_SPEED 0.0f

// Safety Limits
#define MAX_MOTOR_SPEED 255
#define MAX_ANGLE_DEVIATION 30.0f  // degrees
#define EMERGENCY_STOP_ANGLE 45.0f // degrees

// Data Structures
typedef struct {
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

typedef struct {
    float angle_output;
    float speed_output;
    float left_motor_speed;
    float right_motor_speed;
    uint32_t timestamp;
} control_data_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output;
    float output_min;
    float output_max;
} pid_controller_t;

// Global Variables (extern declarations)
extern QueueHandle_t sensor_queue;
extern QueueHandle_t control_queue;
extern SemaphoreHandle_t sensor_mutex;
extern SemaphoreHandle_t control_mutex;

extern sensor_data_t current_sensor_data;
extern control_data_t current_control_data;

// Function Declarations

// Initialization Functions
esp_err_t init_i2c(void);
esp_err_t init_mpu6050(void);
esp_err_t init_motors(void);
esp_err_t init_queues_and_semaphores(void);

// Task Functions
void sensor_task(void *pvParameters);
void control_task(void *pvParameters);
void motor_task(void *pvParameters);
void monitor_task(void *pvParameters);

// Control Functions
void init_pid_controller(pid_controller_t *pid, float kp, float ki, float kd, float min, float max);
float calculate_pid(pid_controller_t *pid, float setpoint, float input, float dt);
void balance_control(sensor_data_t *sensor_data, control_data_t *control_data);

// Motor Control Functions
void set_motor_speed(int motor, float speed);
void emergency_stop(void);

// Utility Functions
float degrees_to_radians(float degrees);
float radians_to_degrees(float radians);
void constrain_float(float *value, float min_val, float max_val);

// Calibration Functions
esp_err_t calibrate_mpu6050(void);

#endif // BALANCING_ROBOT_H
