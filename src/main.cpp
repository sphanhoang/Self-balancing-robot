#include "balance_robot.h"

extern "C" {
	void app_main(void);
}

/* Init angle PID controller */
pid_controller_t pitch_pid = 
{
    .setpoint = 0.0f,
    .kp = KP_PITCH,
    .ki = KI_PITCH,
    .kd = KD_PITCH,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .output = 0.0f
};

pid_controller_t yawRate_pid = 
{
    .setpoint = 0.0f,
    .kp = KP_YAW,
    .ki = KI_YAW,
    .kd = KD_YAW,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .output = 0.0f
};

control_data_t control_data = 
{
    .pitch_output = 0.0f,
    .yawRate_output = 0.0f,
    .left_motor_speed = 0.0f,
    .right_motor_speed = 0.0f,
    .timestamp = 0
};

sensor_data_t sensor_data = 
{
    .yaw = 0.0f,
    .pitch = 0.0f,
    .roll = 0.0f,
    .gyro_x = 0.0f,
    .gyro_y = 0.0f,
    .gyro_z = 0.0f,
    .accel_x = 0.0f,
    .accel_y = 0.0f,
    .accel_z = 0.0f,
    .timestamp = 0
};


////////////////////////            SENSOR SECTION          ////////////////////////
/**
 * @brief Reading data from MPU 6050 task
 */
void sensor_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SENSOR_READ_FREQ_HZ);

    while (1)
    {
        // Read sensor data here
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


////////////////////////            CONTROL SECTION         ////////////////////////

/**
 * @brief PID controller computation
 * @param *pid Pointer to PID variable
 * @param setpoint target value
 * @param feedback sensor feedback value
 * @param dt time difference in seconds
 * @return PID output
 */
float compute_pid(pid_controller_t *pid, float setpoint, float feedback, float dt)
{
    float error = setpoint - feedback;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    pid->output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    pid->previous_error = error;
    return pid->output;
}


/**
 * @brief Balance control loop task
 */
void control_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQ_HZ);
    float dt = 1.0f / CONTROL_LOOP_FREQ_HZ;
    float speed_feedback = (control_data.left_motor_feedback + control_data.right_motor_feedback) / 2.0f;
    while (1)
    {
        // control logic here
        control_data.yawRate_output = compute_pid(&yawRate_pid, 0.0f, sensor_data.yaw, dt);
        control_data.pitch_output = compute_pid(&pitch_pid, 0.0f, sensor_data.pitch, dt);
        control_data.left_motor_speed = control_data.pitch_output + control_data.yawRate_output;
        control_data.right_motor_speed = control_data.pitch_output + + control_data.yawRate_output;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

////////////////////////            MOTOR SECTION           ////////////////////////

/**
 * @brief Initialize pins and PWM channels to drive the motor
 * @note  None
 */ 
esp_err_t motor_init(void)
{
    /* Control pins */
    gpio_config_t motor_io_conf = 
    {
        .pin_bit_mask = (1ULL << MOTOR_IN1_PIN) | (1ULL << MOTOR_IN2_PIN) | 
                        (1ULL << MOTOR_IN3_PIN) | (1ULL << MOTOR_IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&motor_io_conf);
    if (ret != ESP_OK) { return ret; }
        
    /* Init PWM channel */
    mcpwm_config_t pwm_conf =
    {
        .frequency = 200,           /* 200 Hz */
        .cmpr_a = 0,                /* Initial duty cycle */          
        .cmpr_b = 0,                /* Initial duty cycle */
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER
    };

    /* Init MCPWM for Motor A (GPIO 14) */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_ENA_PIN);
    ret = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_conf);
    if (ret != ESP_OK) { return ret; }

    /* Init MCPWM for Motor B (GPIO 32) */
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, MOTOR_ENB_PIN);
    ret = mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_conf);
    if (ret != ESP_OK) { return ret; }
    return ESP_OK;
}

/**
 * @brief Motor control
 * @param motor Motor index (0 = right, 1 = left)
 * @note Pretty self-explanatory isn't it?
 */
void set_motor_speed(int motor, float *speed)
{
    float abs_speed = *speed;
    bool forward = *speed >= 0.0f; 

    // Constrain speed
    if (abs_speed > MAX_MOTOR_SPEED) abs_speed = MAX_MOTOR_SPEED;
    if (abs_speed < MIN_MOTOR_SPEED && abs_speed > 5) abs_speed = MIN_MOTOR_SPEED;


    if (motor == 0) 
    { // Left motor (Motor A)
        // Set direction pins for left motor only
        gpio_set_level((gpio_num_t)MOTOR_IN1_PIN, forward ? 1 : 0);
        gpio_set_level((gpio_num_t)MOTOR_IN2_PIN, forward ? 0 : 1);
        
        // Set PWM duty cycle
        float duty_percent = (abs_speed / 255.0f) * 100.0f;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_percent);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        // ESP_LOGI("MOTOR", "Left motor: speed=%.1f, abs=%.1f, duty=%.1f%%, forward=%d", 
        //         speed, abs_speed, duty_percent, forward);
    }
    else if (motor == 1) 
    { // Right motor (Motor B)
        // Set direction pins for right motor only
        gpio_set_level((gpio_num_t)MOTOR_IN3_PIN, forward ? 0 : 1);
        gpio_set_level((gpio_num_t)MOTOR_IN4_PIN, forward ? 1 : 0);

         // Set PWM duty cycle
        float duty_percent = (abs_speed / 255.0f) * 100.0f;
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty_percent);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
}

/**
 * @brief Drive motor task
 */
void motor_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / MOTOR_UPDATE_FREQ_HZ);
    motor_init();
    while (1)
    {
        set_motor_speed(0, &control_data.right_motor_speed);
        set_motor_speed(1, &control_data.left_motor_speed);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/////////////////////////            MAIN APP SECTION       ////////////////////////
void app_main (void)
{
    xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 10, NULL);
    xTaskCreate(control_task, "control_task", 2048, NULL, 10, NULL);
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 10, NULL);
}