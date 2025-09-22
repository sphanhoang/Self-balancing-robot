#include "balancing_robot.h"
#include <driver/gpio.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

extern "C" {
	void app_main(void);
}

/* Init angle PID controller */
pid_controller_t roll_pid = 
{
    .setpoint = 0.0f,
    .kp = KP_ROLL,
    .ki = KI_ROLL,
    .kd = KD_ROLL,
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
    .roll_output = 0.0f,
    .yawRate_output = 0.0f,
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
uint8_t mpuIntStatus;

// PID Controllers
pid_controller_t angle_pid;
pid_controller_t speed_pid;

// Task Handles
TaskHandle_t sensor_task_handle = NULL;
TaskHandle_t control_task_handle = NULL;
TaskHandle_t motor_task_handle = NULL;
TaskHandle_t monitor_task_handle = NULL;

// Initialization Functions
esp_err_t init_i2c(void) {
    i2c_config_t conf = {};
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
    //   pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    ESP_LOGI(TAG_MPU, "Testing device connections...");
    ESP_LOGI(TAG_MPU, "%s", mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // load and configure the DMP
    ESP_LOGI(TAG_MPU, "Initializing DMP...");
    uint8_t devStatus = mpu.dmpInitialize();

    // calib
    mpu.setXAccelOffset(-2134);
    mpu.setYAccelOffset(-56);
    mpu.setZAccelOffset(1464);
    mpu.setXGyroOffset(-103);
    mpu.setYGyroOffset(155);
    mpu.setZGyroOffset(36);
    // 
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    ESP_LOGI("MPU6050", "Calibration complete!");

    // Enable DMP
	mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    ESP_LOGI("MPU6050", "DMP initialized successfully!");
    return ESP_OK;
}

esp_err_t init_motors(void) {
    // Configure L298 IN1/IN2 direction pins as outputs
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
        .frequency = 20000,         /* 20kHz to reduce noise */
        .cmpr_a = 0,                /* Initial duty cycle */          
        .cmpr_b = 0,                /* Initial duty cycle */
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER
    };

    /* Init MCPWM for Motor A (GPIO 14) */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_ENA_PIN);
    ret = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_conf);
    if (ret != ESP_OK) { 
        ESP_LOGE("MOTORS", "Failed to init MCPWM Unit 0: %s", esp_err_to_name(ret));
        return ret; 
    }

    /* Init MCPWM for Motor B (GPIO 32) */
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, MOTOR_ENB_PIN);
    ret = mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_conf);
    if (ret != ESP_OK) { 
        ESP_LOGE("MOTORS", "Failed to init MCPWM Unit 1: %s", esp_err_to_name(ret));
        return ret; 
    }
    
    // Start the MCPWM timers
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_0);

    ESP_LOGI("MOTORS", "L298 motor pins & PWM configured");
    return ESP_OK;
}

esp_err_t init_queues_and_semaphores(void) {
    sensor_queue = xQueueCreate(10, sizeof(sensor_data_t));  // Increased from 5 to 10
    control_queue = xQueueCreate(10, sizeof(control_data_t)); // Increased from 5 to 10
    sensor_mutex = xSemaphoreCreateMutex();
    control_mutex = xSemaphoreCreateMutex();
    
    if (!sensor_queue || !control_queue || !sensor_mutex || !control_mutex) {
        ESP_LOGE("INIT", "Failed to create queues or semaphores");
        return ESP_FAIL;
    }
    
    ESP_LOGI("INIT", "Queues created: sensor=10, control=10");
    return ESP_OK;
}

// Sensor Task - Reads MPU6050 data using DMP
void sensor_task(void *pvParameters) {
    ESP_LOGI("SENSOR", "Sensor task started");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const uint32_t period_ms = 1000 / SENSOR_READ_FREQ_HZ;
    const TickType_t xFrequency = pdMS_TO_TICKS(period_ms);
    
    ESP_LOGI("SENSOR", "Sensor task frequency: %d Hz, period: %d ms, ticks: %d", 
             SENSOR_READ_FREQ_HZ, period_ms, (int)xFrequency);
    
    // Ensure minimum delay to prevent assertion error
    if (xFrequency <= 0) {
        ESP_LOGE("SENSOR", "Invalid frequency calculation, using minimum delay");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(5)); // 5ms delay as fallback
        }
        return;
    }
    
    while (1) {
	    mpuIntStatus = mpu.getIntStatus();
		fifoCount = mpu.getFIFOCount();

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // Reset FIFO if overflow or full
	        mpu.resetFIFO();
	    } else if (mpuIntStatus & 0x02) {
            // Wait for correct packet size with timeout to prevent watchdog
            uint8_t timeout_count = 0;
            while (fifoCount < packetSize && timeout_count < 10) {
                fifoCount = mpu.getFIFOCount();
                timeout_count++;
                vTaskDelay(pdMS_TO_TICKS(1)); // Yield control
            }
            
            // Skip if timeout occurred
            if (timeout_count >= 10) {
                ESP_LOGW("SENSOR", "FIFO timeout, skipping packet");
                continue;
            }
            
            // Read packet from FIFO
	        mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // Extract quaternion and gravity
	 		mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            // Get raw sensor data
            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            
            // Prepare sensor data
            sensor_data_t sensor_data = {};
            sensor_data.yaw = ypr[0] * 180.0f / M_PI;
            sensor_data.pitch = ypr[1] * 180.0f / M_PI;
            sensor_data.roll = ypr[2] * 180.0f / M_PI;
            sensor_data.gyro_x = gx / 131.0f;  // Convert to deg/s
            sensor_data.gyro_y = gy / 131.0f;
            sensor_data.gyro_z = gz / 131.0f;
            sensor_data.accel_x = ax / 16384.0f;  // Convert to g
            sensor_data.accel_y = ay / 16384.0f;
            sensor_data.accel_z = az / 16384.0f;
            sensor_data.timestamp = xTaskGetTickCount();
            
            // Update global sensor data with mutex protection
            if (xSemaphoreTake(sensor_mutex, portMAX_DELAY)) {
                current_sensor_data = sensor_data;
                xSemaphoreGive(sensor_mutex);
            }
            
            // Output MPU data to serial monitor (commented out to reduce spam)
            // printf("MPU Data - Yaw: %.2f°, Pitch: %.2f°, Roll: %.2f° | Gyro: X=%.2f, Y=%.2f, Z=%.2f | Accel: X=%.2f, Y=%.2f, Z=%.2f\n",
            //        sensor_data.yaw, sensor_data.pitch, sensor_data.roll,
            //        sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z,
            //        sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
            
            // Send to control queue with timeout to prevent blocking
            if (xQueueSend(control_queue, &sensor_data, 10 / portTICK_PERIOD_MS) != pdTRUE) {
                // Only log warning occasionally to reduce spam
                static uint32_t last_warning = 0;
                uint32_t now = xTaskGetTickCount();
                if (now - last_warning > pdMS_TO_TICKS(1000)) { // Log max once per second
                    ESP_LOGW("SENSOR", "Control queue full, dropping sensor data");
                    last_warning = now;
                }
            }
        }
        
        // Use simple delay instead of vTaskDelayUntil for sensor task
        vTaskDelay(pdMS_TO_TICKS(20)); // 20ms delay (50Hz max) to prevent watchdog timeout
    }
}

/**
 * @brief Balance control loop task
 */
void control_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQ_HZ);
    
    sensor_data_t sensor_data;
    control_data_t control_data;
    
    while (1) {
        // Process one sensor data per loop iteration
        if (xQueueReceive(control_queue, &sensor_data, 0) == pdTRUE) {
            // Implement balance control
            balance_control(&sensor_data, &control_data);
            
            // Update global control data with mutex protection
            if (xSemaphoreTake(control_mutex, 10 / portTICK_PERIOD_MS)) {
                current_control_data = control_data;
                xSemaphoreGive(control_mutex);
            }
        }
        
        // Use proper timing to prevent watchdog timeout
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Motor Task - Controls motor speeds
void motor_task(void *pvParameters) {
    ESP_LOGI("MOTOR", "Motor task started");
    
    // Use simple delay instead of vTaskDelayUntil to reduce stack usage
    const TickType_t delay_ticks = pdMS_TO_TICKS(1000 / MOTOR_UPDATE_FREQ_HZ);
    
    while (1) {
        // Get control data with timeout to prevent blocking
        if (xSemaphoreTake(control_mutex, 100 / portTICK_PERIOD_MS)) {
            // Apply motor control directly without copying data
            set_motor_speed(0, current_control_data.left_motor_speed);   // Left motor
            set_motor_speed(1, current_control_data.right_motor_speed);  // Right motor
            xSemaphoreGive(control_mutex);
        }
        
        vTaskDelay(delay_ticks);
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
        printf("ROLL: %3.1f, PID integral: %3.1f, Control Roll output: %3.1f \n", sensor_data.roll, roll_pid.integral, control_data.roll_output);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Control Functions
void init_pid_controller(pid_controller_t *pid, float kp, float ki, float kd, float min, float max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
    pid->output_min = min;
    pid->output_max = max;
}

float calculate_pid(pid_controller_t *pid, float setpoint, float input, float dt) {
    float error = setpoint - input;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term
    pid->integral += error * dt;
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float d_term = pid->kd * (error - pid->previous_error) / dt;
    
    // Calculate output
    pid->output = p_term + i_term + d_term;
    
    // Constrain output
    constrain_float(&pid->output, pid->output_min, pid->output_max);
    
    // Update previous error
    pid->previous_error = error;
    
    return pid->output;
}

void balance_control(sensor_data_t *sensor_data, control_data_t *control_data) {
    static float dt = 1.0f / CONTROL_LOOP_FREQ_HZ;
    
    // Emergency stop if angle is too large
    if (fabs(sensor_data->pitch) > EMERGENCY_STOP_ANGLE) {
        control_data->left_motor_speed = 0.0f;
        control_data->right_motor_speed = 0.0f;
        ESP_LOGW("CONTROL", "Emergency stop! Angle: %.2f°", sensor_data->pitch);
        return;
    }
        
    // Calculate speed PID output (if you have encoders)
    float speed_output = calculate_pid(&speed_pid, 0.0f, 0.0f, dt); // Placeholder
    
    // Calculate angle PID output (pitch control)
    float angle_output = calculate_pid(&angle_pid, 0.0f, sensor_data->pitch, dt);

    
    // Combine outputs
    float left_speed = angle_output + speed_output;
    float right_speed = angle_output + speed_output;
    
    // Debug output (commented out to reduce spam)
    // ESP_LOGD("CONTROL", "Pitch: %.2f°, Angle PID: %.1f, Left: %.1f, Right: %.1f", 
    //          sensor_data->pitch, angle_output, left_speed, right_speed);
    
    // Constrain motor speeds
    constrain_float(&left_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    constrain_float(&right_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    control_data->angle_output = angle_output;
    control_data->speed_output = speed_output;
    control_data->left_motor_speed = left_speed;
    control_data->right_motor_speed = right_speed;
    control_data->timestamp = xTaskGetTickCount();
}

// Motor Control Functions
void set_motor_speed(int motor, float speed) {
    // Convert speed (-255 to 255) to PWM and direction
    float abs_speed = fabs(speed);
    bool forward = speed >= 0.0f;
    
    // Constrain speed
    if (abs_speed > MAX_MOTOR_SPEED) abs_speed = MAX_MOTOR_SPEED;
    if (abs_speed < MIN_MOTOR_SPEED && abs_speed > 0) abs_speed = MIN_MOTOR_SPEED;
    
    if (motor == 0) { // Left motor (Motor A)
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
    else if (motor == 1) { // Right motor (Motor B)
        // Set direction pins for right motor only
        gpio_set_level((gpio_num_t)MOTOR_IN3_PIN, forward ? 1 : 0);
        gpio_set_level((gpio_num_t)MOTOR_IN4_PIN, forward ? 0 : 1);
        
        // Set PWM duty cycle
        float duty_percent = (abs_speed / 255.0f) * 100.0f;
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty_percent);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        
        // ESP_LOGI("MOTOR", "Right motor: speed=%.1f, abs=%.1f, duty=%.1f%%, forward=%d", 
        //         speed, abs_speed, duty_percent, forward);
    }
}

void emergency_stop(void) {
    set_motor_speed(0, 0.0f);
    set_motor_speed(1, 0.0f);
    ESP_LOGW("MOTOR", "Emergency stop activated!");
}

// Test function to verify motor control
void test_motors(void) {
    ESP_LOGI("MOTOR", "Testing motors...");
    
    // Test with higher speeds to ensure motors move
    ESP_LOGI("MOTOR", "Left motor forward 80%");
    set_motor_speed(0, 200.0f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI("MOTOR", "Left motor reverse 80%");
    set_motor_speed(0, -200.0f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI("MOTOR", "Right motor forward 80%");
    set_motor_speed(1, 200.0f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI("MOTOR", "Right motor reverse 80%");
    set_motor_speed(1, -200.0f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Stop both motors
    ESP_LOGI("MOTOR", "Stopping motors");
    set_motor_speed(0, 0.0f);
    set_motor_speed(1, 0.0f);
    
    ESP_LOGI("MOTOR", "Motor test complete");
}

// Utility Functions
float degrees_to_radians(float degrees) {
    return degrees * M_PI / 180.0f;
}

float radians_to_degrees(float radians) {
    return radians * 180.0f / M_PI;
}

void constrain_float(float *value, float min_val, float max_val) {
    if (*value < min_val) *value = min_val;
    if (*value > max_val) *value = max_val;
}

// Main Application
void app_main(void) {
    ESP_LOGI("MAIN", "Starting Self-Balancing Robot...");
    
    // Initialize all components
    ESP_ERROR_CHECK(init_i2c());
    ESP_ERROR_CHECK(init_mpu6050());
    ESP_ERROR_CHECK(init_motors());
    ESP_ERROR_CHECK(init_queues_and_semaphores());
    
    // Initialize PID controllers
    init_pid_controller(&angle_pid, KP_ANGLE, KI_ANGLE, KD_ANGLE, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    init_pid_controller(&speed_pid, KP_SPEED, KI_SPEED, KD_SPEED, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    // Create tasks
    xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, &sensor_task_handle);
    xTaskCreate(control_task, "control_task", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, &control_task_handle);
    xTaskCreate(motor_task, "motor_task", MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, &motor_task_handle);
    xTaskCreate(monitor_task, "monitor_task", MONITOR_TASK_STACK_SIZE, NULL, MONITOR_TASK_PRIORITY, &monitor_task_handle);
    
    ESP_LOGI("MAIN", "All tasks created successfully!");
    
    // Test motors before starting balancing
    test_motors();
    
    ESP_LOGI("MAIN", "Self-balancing robot is running...");
    ESP_LOGI("MAIN", "MPU Data Format: Yaw, Pitch, Roll (degrees) | Gyro X,Y,Z (deg/s) | Accel X,Y,Z (g)");
    ESP_LOGI("MAIN", "Monitor will show detailed status every second");
}