#include "balancing_robot.h"
#include <driver/gpio.h>
#include <driver/ledc.h>

extern "C" {
	void app_main(void);
}

// Global Variables
QueueHandle_t sensor_queue;
QueueHandle_t control_queue;
SemaphoreHandle_t sensor_mutex;
SemaphoreHandle_t control_mutex;

sensor_data_t current_sensor_data;
control_data_t current_control_data;

// MPU6050 instance
MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
float ypr[3];
uint16_t packetSize = 42;
uint16_t fifoCount;
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

// LEDC (PWM) configuration for L298
static const ledc_mode_t kPwmMode = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t kPwmTimer = LEDC_TIMER_0;
static const ledc_timer_bit_t kPwmResolution = LEDC_TIMER_8_BIT; // 0..255
static const uint32_t kPwmFreqHz = 20000; // 20 kHz to reduce audible noise
static const ledc_channel_t kLeftPwmChannel = LEDC_CHANNEL_0;
static const ledc_channel_t kRightPwmChannel = LEDC_CHANNEL_1;

// Initialization Functions
esp_err_t init_i2c(void) {
    i2c_config_t conf = {};
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
    conf.scl_io_num = (gpio_num_t)PIN_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    
    esp_err_t ret = i2c_param_config(I2C_NUM, &conf);
    if (ret != ESP_OK) return ret;
    
    return i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t init_mpu6050(void) {
	mpu.initialize();
    
    if (!mpu.testConnection()) {
        ESP_LOGE("MPU6050", "MPU6050 connection failed!");
        return ESP_FAIL;
    }
    
    ESP_LOGI("MPU6050", "MPU6050 connection successful!");
    
    // Initialize DMP
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
        ESP_LOGE("MPU6050", "DMP initialization failed (code %d)", devStatus);
        return ESP_FAIL;
    }
    
    // Calibrate sensors
    ESP_LOGI("MPU6050", "Calibrating sensors...");
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
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_LEFT_IN1_PIN) | (1ULL << MOTOR_LEFT_IN2_PIN) |
                           (1ULL << MOTOR_RIGHT_IN1_PIN) | (1ULL << MOTOR_RIGHT_IN2_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;

    // Default direction low
    gpio_set_level((gpio_num_t)MOTOR_LEFT_IN1_PIN, 0);
    gpio_set_level((gpio_num_t)MOTOR_LEFT_IN2_PIN, 0);
    gpio_set_level((gpio_num_t)MOTOR_RIGHT_IN1_PIN, 0);
    gpio_set_level((gpio_num_t)MOTOR_RIGHT_IN2_PIN, 0);

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = kPwmMode;
    ledc_timer.timer_num = kPwmTimer;
    ledc_timer.duty_resolution = kPwmResolution;
    ledc_timer.freq_hz = kPwmFreqHz;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) return ret;

    // Configure LEDC channels for PWM pins
    ledc_channel_config_t ch_left = {};
    ch_left.gpio_num = MOTOR_LEFT_PWM_PIN;
    ch_left.speed_mode = kPwmMode;
    ch_left.channel = kLeftPwmChannel;
    ch_left.intr_type = LEDC_INTR_DISABLE;
    ch_left.timer_sel = kPwmTimer;
    ch_left.duty = 0;
    ch_left.hpoint = 0;
    ret = ledc_channel_config(&ch_left);
    if (ret != ESP_OK) return ret;

    ledc_channel_config_t ch_right = {};
    ch_right.gpio_num = MOTOR_RIGHT_PWM_PIN;
    ch_right.speed_mode = kPwmMode;
    ch_right.channel = kRightPwmChannel;
    ch_right.intr_type = LEDC_INTR_DISABLE;
    ch_right.timer_sel = kPwmTimer;
    ch_right.duty = 0;
    ch_right.hpoint = 0;
    ret = ledc_channel_config(&ch_right);
    if (ret != ESP_OK) return ret;

    ESP_LOGI("MOTORS", "L298 motor pins & PWM configured");
    return ESP_OK;
}

esp_err_t init_queues_and_semaphores(void) {
    sensor_queue = xQueueCreate(5, sizeof(sensor_data_t));
    control_queue = xQueueCreate(5, sizeof(control_data_t));
    sensor_mutex = xSemaphoreCreateMutex();
    control_mutex = xSemaphoreCreateMutex();
    
    if (!sensor_queue || !control_queue || !sensor_mutex || !control_mutex) {
        ESP_LOGE("INIT", "Failed to create queues or semaphores");
        return ESP_FAIL;
    }
    
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
            
            // Output MPU data to serial monitor
            printf("MPU Data - Yaw: %.2f°, Pitch: %.2f°, Roll: %.2f° | Gyro: X=%.2f, Y=%.2f, Z=%.2f | Accel: X=%.2f, Y=%.2f, Z=%.2f\n",
                   sensor_data.yaw, sensor_data.pitch, sensor_data.roll,
                   sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z,
                   sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
            
            // Send to control queue
            if (xQueueSend(control_queue, &sensor_data, 0) != pdTRUE) {
                ESP_LOGW("SENSOR", "Control queue full, dropping sensor data");
            }
        }
        
        // Use simple delay instead of vTaskDelayUntil for sensor task
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay (100Hz max) to prevent watchdog timeout
    }
}

// Control Task - Implements PID control for balancing
void control_task(void *pvParameters) {
    ESP_LOGI("CONTROL", "Control task started");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQ_HZ);
    
    // Ensure minimum delay to prevent assertion error
    if (xFrequency <= 0) {
        ESP_LOGE("CONTROL", "Invalid frequency calculation, using minimum delay");
        vTaskDelay(pdMS_TO_TICKS(1));
        return;
    }
    
    sensor_data_t sensor_data;
    control_data_t control_data;
    
    while (1) {
        // Wait for sensor data
        if (xQueueReceive(control_queue, &sensor_data, portMAX_DELAY) == pdTRUE) {
            // Implement balance control
            balance_control(&sensor_data, &control_data);
            
            // Update global control data with mutex protection
            if (xSemaphoreTake(control_mutex, portMAX_DELAY)) {
                current_control_data = control_data;
                xSemaphoreGive(control_mutex);
            }
            
            // Send to motor task
            if (xQueueSend(control_queue, &control_data, 0) != pdTRUE) {
                ESP_LOGW("CONTROL", "Motor queue full, dropping control data");
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Motor Task - Controls motor speeds
void motor_task(void *pvParameters) {
    ESP_LOGI("MOTOR", "Motor task started");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / MOTOR_UPDATE_FREQ_HZ);
    
    // Ensure minimum delay to prevent assertion error
    if (xFrequency <= 0) {
        ESP_LOGE("MOTOR", "Invalid frequency calculation, using minimum delay");
        vTaskDelay(pdMS_TO_TICKS(1));
        return;
    }
    
    control_data_t control_data;
    
    while (1) {
        // Get control data
        if (xSemaphoreTake(control_mutex, portMAX_DELAY)) {
            control_data = current_control_data;
            xSemaphoreGive(control_mutex);
        }
        
        // Apply motor control
        set_motor_speed(0, control_data.left_motor_speed);   // Left motor
        set_motor_speed(1, control_data.right_motor_speed);  // Right motor
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Monitor Task - Logs system status
void monitor_task(void *pvParameters) {
    ESP_LOGI("MONITOR", "Monitor task started");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1Hz
    
    while (1) {
        sensor_data_t sensor_data;
        control_data_t control_data;
        
        // Get current data
        if (xSemaphoreTake(sensor_mutex, 100 / portTICK_PERIOD_MS)) {
            sensor_data = current_sensor_data;
            xSemaphoreGive(sensor_mutex);
        }
        
        if (xSemaphoreTake(control_mutex, 100 / portTICK_PERIOD_MS)) {
            control_data = current_control_data;
            xSemaphoreGive(control_mutex);
        }
        
        // Log detailed status
        ESP_LOGI("MONITOR", "=== Robot Status ===");
        ESP_LOGI("MONITOR", "Attitude - Yaw: %.2f°, Pitch: %.2f°, Roll: %.2f°", 
                sensor_data.yaw, sensor_data.pitch, sensor_data.roll);
        ESP_LOGI("MONITOR", "Gyro - X: %.2f°/s, Y: %.2f°/s, Z: %.2f°/s", 
                sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
        ESP_LOGI("MONITOR", "Accel - X: %.2fg, Y: %.2fg, Z: %.2fg", 
                sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
        ESP_LOGI("MONITOR", "Control - Angle: %.1f, Speed: %.1f, Left: %.1f, Right: %.1f",
                control_data.angle_output, control_data.speed_output,
                control_data.left_motor_speed, control_data.right_motor_speed);
        
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
    float angle_output = calculate_pid(&angle_pid, speed_output, sensor_data->pitch, dt);

    
    // Combine outputs
    float left_speed = angle_output + speed_output;
    float right_speed = angle_output + speed_output;
    
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
    // Convert speed (-255 to 255) to PWM and direction using IN1/IN2
    int pwm_value = (int)fabs(speed);
    if (pwm_value > MAX_MOTOR_SPEED) pwm_value = MAX_MOTOR_SPEED;
    bool forward = speed >= 0.0f;

    if (motor == 0) { // Left motor
        // Direction
        gpio_set_level((gpio_num_t)MOTOR_LEFT_IN1_PIN, forward ? 1 : 0);
        gpio_set_level((gpio_num_t)MOTOR_LEFT_IN2_PIN, forward ? 0 : 1);
        // PWM
        ledc_set_duty(kPwmMode, kLeftPwmChannel, pwm_value);
        ledc_update_duty(kPwmMode, kLeftPwmChannel);
    } else { // Right motor
        gpio_set_level((gpio_num_t)MOTOR_RIGHT_IN1_PIN, forward ? 1 : 0);
        gpio_set_level((gpio_num_t)MOTOR_RIGHT_IN2_PIN, forward ? 0 : 1);
        ledc_set_duty(kPwmMode, kRightPwmChannel, pwm_value);
        ledc_update_duty(kPwmMode, kRightPwmChannel);
    }
}

void emergency_stop(void) {
    set_motor_speed(0, 0.0f);
    set_motor_speed(1, 0.0f);
    ESP_LOGW("MOTOR", "Emergency stop activated!");
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
    ESP_LOGI("MAIN", "Self-balancing robot is running...");
    ESP_LOGI("MAIN", "MPU Data Format: Yaw, Pitch, Roll (degrees) | Gyro X,Y,Z (deg/s) | Accel X,Y,Z (g)");
    ESP_LOGI("MAIN", "Monitor will show detailed status every second");
}
