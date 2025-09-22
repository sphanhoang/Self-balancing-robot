/**
 * @file main.cpp
 * @brief Self-balancing robot control logic
 * @author Son Phan
 * @date Sept 2025
 */

#include "balancing_robot.h"

static const char *TAG_MPU = "MPU";

extern "C" {
	void app_main(void);
}

/* Init PID controllers*/


pid_controller_t speed_pid = 
{
    .setpoint = 0.0f,
    .kp = KP_SPEED,
    .ki = KI_SPEED,
    .kd = KD_SPEED,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .output = 0.0f
};

pid_controller_t roll_pid = 
{
    .setpoint = 0.5f,
    .kp = KP_ROLL,
    .ki = KI_ROLL,
    .kd = KD_ROLL,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .output = 0.0f
};

control_data_t control_data = 
{
    .roll_output = 0.0f,
    .speed_output = 0.0f,
    .left_motor_speed = 0.0f,
    .right_motor_speed = 0.0f,
    .left_motor_feedback = 0.0f,
    .right_motor_feedback = 0.0f,
    .timestamp = 0
};

sensor_data_t sensor_data = 
{
    .yaw = 0.0f,
    .pitch = 0.0f,
    .roll = 0.0f,
    // .gyro_x = 0.0f,
    // .gyro_y = 0.0f,
    // .gyro_z = 0.0f,
    .accel_x = 0.0f,
    .accel_y = 0.0f,
    .accel_z = 0.0f,
    .timestamp = 0
};

uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;           /* [w, x, y, z]         quaternion container */
VectorFloat gravity;    /* [x, y, z]            gravity vector */
float ypr[3];           /* [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector */
VectorInt16 aa, aaReal;        /* [x, y, z]            accel sensor measurements */
// uint8_t e_stop;

////////////////////////            SENSOR SECTION          ////////////////////////
void initI2C(void) 
{
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

/**
 * @brief MPU 6050 init
 */
void mpu_setup(MPU6050 &mpu)
{
    initI2C();
    mpu.initialize();
    // pinMode(INTERRUPT_PIN, INPUT);

    /* verify connection */
    ESP_LOGI(TAG_MPU, "Testing device connections...");
    ESP_LOGI(TAG_MPU, "%s", mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    /* load and configure the DMP */
    ESP_LOGI(TAG_MPU, "Initializing DMP...");
    uint8_t devStatus = mpu.dmpInitialize();

    /* calib */
    mpu.setXAccelOffset(-2134);
    mpu.setYAccelOffset(-56);
    mpu.setZAccelOffset(1464);
    mpu.setXGyroOffset(-103);
    mpu.setYGyroOffset(155);
    mpu.setZGyroOffset(36);
    /**/ 
    // mpu.CalibrateAccel(6);
    // mpu.CalibrateGyro(6);

    /* make sure it worked (returns 0 if so) */
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        ESP_LOGI(TAG_MPU, "Enabling DMP...");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        // mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // ESP_LOGI(TAG_MPU, "DMP ready! Waiting for first interrupt...");
        // dmpReady = true;

        /* get expected DMP packet size for later comparison */
        packetSize = mpu.dmpGetFIFOPacketSize();
        ESP_LOGI(TAG_MPU, "DMP is ready!");
    } 
    else 
    {
        /* ERROR!
        1 = initial memory load failed
        2 = DMP configuration updates failed
        (if it's going to break, usually the code will be 1) */
        ESP_LOGE(TAG_MPU, "DMP Initialization failed (code ");
        ESP_LOGI(TAG_MPU, "%d", devStatus);
        ESP_LOGI(TAG_MPU, ")");
    } 
    ESP_LOGI(TAG_MPU, "MPU 6050 setup complete.");
}   


/**
 * @brief Reading data from MPU 6050 task
 */
void sensor_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SENSOR_READ_FREQ_HZ);
    MPU6050 mpu = MPU6050();
    mpu_setup(mpu);
    while (1)
    {
        /* Read sensor data here
        Clear the buffer so as we can get fresh values
        The sensor is running a lot faster than our sample period */
        mpu.resetFIFO();
        
        // get current FIFO count
        uint16_t fifoCount = mpu.getFIFOCount();
        
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        sensor_data.yaw = ypr[0] * 180/M_PI;
        sensor_data.pitch = ypr[1] * 180/M_PI;
        sensor_data.roll = ypr[2] * 180/M_PI;
        sensor_data.accel_x = aaReal.x;
        sensor_data.accel_y = aaReal.y;
        sensor_data.accel_z = aaReal.z;
        sensor_data.timestamp = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(NULL);
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
float compute_pid(pid_controller_t *pid, float feedback, float dt)
{
    if (pid == &roll_pid)
    {
        #define PID_INTEGRAL_CLAMP ROLL_PID_INTEGRAL_CLAMP
        #define PID_OUTPUT_CLAMP ROLL_PID_OUTPUT_CLAMP
    }
    else if (pid == &speed_pid)
    {
        #undef PID_INTEGRAL_CLAMP
        #undef PID_OUTPUT_CLAMP
        #define PID_INTEGRAL_CLAMP SPEED_PID_INTEGRAL_CLAMP
        #define PID_OUTPUT_CLAMP SPEED_PID_OUTPUT_CLAMP
    }
    else
    {
        // Unknown PID controller
        return 0.0f;
    }
    float error = pid->setpoint - feedback;
    pid->integral += error * dt;
    pid->integral = pid->integral > PID_INTEGRAL_CLAMP ? PID_INTEGRAL_CLAMP : (pid->integral < -PID_INTEGRAL_CLAMP ? -PID_INTEGRAL_CLAMP : pid->integral);
    float derivative = (error - pid->previous_error) / dt;
    pid->output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    pid->output = pid->output > PID_OUTPUT_CLAMP ? PID_OUTPUT_CLAMP : (pid->output < -PID_OUTPUT_CLAMP ? -PID_OUTPUT_CLAMP : pid->output);
    pid->previous_error = error;
    return pid->output;
}

/**
 * @brief Balance control loop task
 */
void balance_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQ_HZ);
    sensor_data_t current_data = sensor_data;
    float dt = 1.0f / CONTROL_LOOP_FREQ_HZ;
    // float velocity = current_data.accel_y * dt;
    while (1)
    {
        /* control logic here */
        if (current_data.roll > MAX_ANGLE || current_data.roll < MIN_ANGLE)
        {
            control_data.speed_output = 0.0f;
            control_data.roll_output = 0.0f;
        }
        else
        {
        //    control_data.speed_output = compute_pid(&speed_pid, current_data.accel_y, dt);
            control_data.roll_output = compute_pid(&roll_pid, current_data.roll, dt);
        }
        // control_data.yawRate_output = compute_pid(&yawRate_pid, 0.0f, current_data.yaw, dt);
        control_data.left_motor_speed = control_data.roll_output + control_data.speed_output;
        control_data.right_motor_speed = control_data.roll_output + control_data.speed_output;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(NULL);
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
        .frequency = PWM_FREQ_HZ,           /* 200 Hz */
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
    float abs_speed = *speed > 0 ? *speed : -*speed;
    bool is_forward = *speed >= 0.0f; 

    /* Constrain speed */
    if (abs_speed > MAX_MOTOR_SPEED) abs_speed = MAX_MOTOR_SPEED;
    if (abs_speed < MIN_MOTOR_SPEED && abs_speed > 0) abs_speed = MIN_MOTOR_SPEED;

    if (motor == 0) 
    {   /* Right motor (Motor A) */
        /* Set direction pins for Right motor only */
        gpio_set_level((gpio_num_t)MOTOR_IN1_PIN, is_forward ? 0 : 1);
        gpio_set_level((gpio_num_t)MOTOR_IN2_PIN, is_forward ? 1 : 0);
        
        /* Set PWM duty cycle */
        float duty_percent = (abs_speed / 255.0f) * 100.0f;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_percent);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        // ESP_LOGI("MOTOR", "Right motor: speed=%.1f, abs=%.1f, duty=%.1f%%, is_forward=%d", 
        //         speed, abs_speed, duty_percent, is_forward);
    }
    else if (motor == 1) 
    {   /* Left motor (Motor B) */
        /* Set direction pins for Left motor only */
        gpio_set_level((gpio_num_t)MOTOR_IN3_PIN, is_forward ? 1 : 0);
        gpio_set_level((gpio_num_t)MOTOR_IN4_PIN, is_forward ? 0 : 1);

         /* Set PWM duty cycle */
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
    vTaskDelete(NULL);
}
/////////////////////////            Monitor Task           ////////////////////////

void monitor_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 1 second
    while (1)
    {
        printf("ROLL: %3.1f, Accel: %3.1f, speed PID Integral: %3.1f, PWM output: %3.1f \n", sensor_data.roll, sensor_data.accel_y, speed_pid.integral, control_data.left_motor_speed);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(NULL);
}
/////////////////////////            MAIN APP SECTION       ////////////////////////

void app_main (void)
{
    xTaskCreate(monitor_task, "monitor_task", 2048, NULL, 10, NULL);
    xTaskCreate(sensor_task, "sensor_task", 2048*2, NULL, 10, NULL);
    xTaskCreate(balance_task, "balance_task", 2048, NULL, 10, NULL);
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 10, NULL);
}