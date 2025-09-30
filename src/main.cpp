/**
 * @file main.cpp
 * @brief Self-balancing robot control logic
 * @author Son Phan
 * @date Sept 2025
 */

#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "balancing_robot.h"

static const char *TAG_MPU = "MPU";
static const char *TAG_MUTEX = "MUTEX";

extern "C" {
	void app_main(void);
}

/* Init PID controllers*/

pid_controller_t speed_pid = 
{
    .setpoint = TILT_ANGLE,
    .error = 0.0f,
    .kp = KP_SPEED,
    .ki = KI_SPEED,
    .kd = KD_SPEED,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .output = 0.0f,
    .integral_clamp = 0.0f,
    .output_clamp = 0.0f
};

pid_controller_t roll_pid = 
{
    .setpoint = 0.0f,
    .error = 0.0f,
    .kp = KP_ROLL,
    .ki = KI_ROLL,
    .kd = KD_ROLL,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .output = 0.0f,
    .integral_clamp = 0.0f,
    .output_clamp = 0.0f
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
    .timestamp = 0,
    .dt_since_last = 0.0f
};

encoder_data_t encoder_data = {};

SemaphoreHandle_t mutex_sensor_data;
SemaphoreHandle_t mutex_control_data;
SemaphoreHandle_t mutex_encoder_data;
SemaphoreHandle_t mutex_web_data;

static QueueHandle_t interrupt_queue = NULL; 

uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;           /* [w, x, y, z]         quaternion container */
VectorFloat gravity;    /* [x, y, z]            gravity vector */
float ypr[3];           /* [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector */
VectorInt16 aa, aaReal;        /* [x, y, z]            accel sensor measurements */
// uint8_t e_stop;


////////////////////////////////////////////////            SENSOR SECTION          ////////////////////////////////////////////////


void initI2C(void) 
{
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

/**
 * @brief MPU 6050 init
 */
void mpu_setup(MPU6050 &mpu)
{
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
    mpu.setRate(10);
    ESP_LOGI(TAG_MPU, "MPU 6050 setup complete.");
}   


/**
 * @brief Reading data from MPU 6050 (will add encoder reading later)
 */
void sensor_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SENSOR_READ_FREQ_HZ);
    sensor_data_t new_data;
    MPU6050 mpu = MPU6050();
    initI2C();
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
        new_data.yaw = ypr[0] * 180/M_PI;
        new_data.pitch = ypr[1] * 180/M_PI;
        new_data.roll = ypr[2] * 180/M_PI;
        new_data.accel_x = aaReal.x;
        new_data.accel_y = aaReal.y;
        new_data.accel_z = aaReal.z;
        new_data.timestamp = esp_timer_get_time();
        if (xSemaphoreTake(mutex_sensor_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            sensor_data = new_data;
            xSemaphoreGive(mutex_sensor_data);
        }
        else
        {
            ESP_LOGW("SENSOR", "Failed to take mutex");
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(NULL);
}


////////////////////////////////////////////////            CONTROL SECTION         ////////////////////////////////////////////////


/**
 * @brief PID controller computation
 * @param *pid Pointer to PID variable
 * @param *feedback Pointer to sensor variable
 * @param dt time difference in seconds
 * @return PID output
 */
float compute_pid(pid_controller_t *pid, float *feedback, float dt)
{
    if (pid == &roll_pid)
    {
        roll_pid.integral_clamp = ROLL_PID_INTEGRAL_CLAMP;
        roll_pid.output_clamp = ROLL_PID_OUTPUT_CLAMP;
    }
    else if (pid == &speed_pid)
    {
        speed_pid.integral_clamp = SPEED_PID_INTEGRAL_CLAMP;
        speed_pid.output_clamp = SPEED_PID_OUTPUT_CLAMP;
    }
    else
    {
        // Unknown PID controller
        return 0.0f;
    }
    pid->error = pid->setpoint - *feedback;
    pid->integral += pid->error * dt;
    pid->integral = pid->integral > pid->integral_clamp ? 
                    pid->integral_clamp : (pid->integral < -pid->integral_clamp ? 
                    -pid->integral_clamp : pid->integral);
    float derivative = (pid->error - pid->previous_error) / dt;
    pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    pid->output = pid->output > pid->output_clamp ? 
                    pid->output_clamp : (pid->output < -pid->output_clamp ? 
                    -pid->output_clamp : pid->output);
    pid->previous_error = pid->error;
    return pid->output;
}

/**
 * @brief Balance control loop task
 */
void balance_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQ_HZ);
    sensor_data_t current_data = {};
    control_data_t local_control_data = {};
    web_control_t web_ctrl = {};
    static uint32_t debug_count;
    float dt = 1.0f / CONTROL_LOOP_FREQ_HZ;
    while (1)
    {  
        // Get web control parameters safely
        if (debug_count++ % 200 == 0) 
        {
            if (xSemaphoreTake(mutex_web_data, pdMS_TO_TICKS(10)) == pdTRUE) 
            {
                web_ctrl = web_control;
                xSemaphoreGive(mutex_web_data);
                // Debug logging every 2 seconds (200 iterations at 100Hz)

                // ESP_LOGI("BALANCE", "Web params - KP: %.2f, KI: %.2f, KD: %.2f, Target: %.2f, Enable: %d",
                //         web_ctrl.kp_roll, web_ctrl.ki_roll, web_ctrl.kd_roll, 
                //         web_ctrl.target_angle, web_ctrl.enable_balance);
            }
        }
        // Update PID parameters from web interface
        roll_pid.kp = web_ctrl.kp_roll;
        roll_pid.ki = web_ctrl.ki_roll;
        roll_pid.kd = web_ctrl.kd_roll;
        roll_pid.setpoint = web_ctrl.target_angle;

        if (xSemaphoreTake(mutex_sensor_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (current_data.timestamp > 0)
            {
                dt = (sensor_data.timestamp - current_data.timestamp) / 1000000.0f;
                dt = dt < 0.005f ? 0.005f : (dt > 0.020f ? 0.020f : dt);
            }
            current_data = sensor_data;
            xSemaphoreGive(mutex_sensor_data);
        }
        else
        {
            ESP_LOGW(TAG_MUTEX, "Failed to take sensor data mutex");
        }
        if (xSemaphoreTake(mutex_control_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            local_control_data = control_data;
            xSemaphoreGive(mutex_control_data);
        }
        else
        {
            ESP_LOGW(TAG_MUTEX, "Failed to take control data mutex");
        }
        if (xSemaphoreTake(mutex_encoder_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            local_control_data.left_motor_feedback = encoder_data.left_motor_speed;
            local_control_data.right_motor_feedback = encoder_data.right_motor_speed;
            xSemaphoreGive(mutex_encoder_data);
        }
        else
        {
            ESP_LOGW(TAG_MUTEX, "Failed to take encoder data mutex");
        }

        /* control logic here */
        if (current_data.roll > MAX_ANGLE || current_data.roll < MIN_ANGLE)
        {
            roll_pid.integral = 0.0f;
            speed_pid.integral = 0.0f;
            local_control_data.speed_output = 0.0f;
            local_control_data.roll_output = 0.0f;
        }
        else
        {
            // local_control_data.speed_output = compute_pid(&speed_pid, &speed_feedback, dt); /* WIP */
            local_control_data.roll_output = compute_pid(&roll_pid, &current_data.roll, dt);
        }
        local_control_data.left_motor_speed = local_control_data.roll_output + local_control_data.speed_output;
        local_control_data.right_motor_speed = local_control_data.roll_output + local_control_data.speed_output;

        if (xSemaphoreTake(mutex_control_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            control_data = local_control_data;
            xSemaphoreGive(mutex_control_data);
        }
        else
        {
            ESP_LOGW(TAG_MUTEX, "Failed to take control data mutex");
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(NULL);
}


////////////////////////////////////////////////            MOTOR SECTION           ////////////////////////////////////////////////


/**
 * @brief Initialize pins and PWM channels to drive the motor
 * @note  None
 */ 
esp_err_t motor_init(void)
{
    /* Control pins set to output mode */
    gpio_config_t motor_io_conf = 
    {
        .pin_bit_mask = GPIO_MOTOR_PIN_SEL,
        /* set as output mode */
        .mode = GPIO_MODE_OUTPUT,
        /* disable pull-up resistor */
        .pull_up_en = GPIO_PULLUP_DISABLE,
        /* disable pull-down resistor */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        /* disable interrupt */
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&motor_io_conf);
    if (ret != ESP_OK) { return ret; }
        
    /* Setup PWM channel ENA and ENB */
    mcpwm_config_t pwm_conf =
    {
        .frequency = PWM_FREQ_HZ,           /* 100 Hz */
        .cmpr_a = 0,                /* Initial duty cycle */          
        .cmpr_b = 0,                /* Initial duty cycle */
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER
    };

    /* Init MCPWM for Right Motor A (GPIO 14) */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_ENA_PIN);
    ret = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_conf);
    if (ret != ESP_OK) { return ret; }

    /* Init MCPWM for Left Motor B (GPIO 32) */
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
    float left_speed, right_speed;
    while (1)
    {
        if (xSemaphoreTake(mutex_control_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            left_speed = control_data.left_motor_speed;
            right_speed = control_data.right_motor_speed;
            xSemaphoreGive(mutex_control_data);
        }
        else
        {
            ESP_LOGW(TAG_MUTEX, "Failed to take control data mutex");
        }
        set_motor_speed(0, &left_speed);
        set_motor_speed(1, &right_speed);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(NULL);
}


/////////////////////////////////////////////////               Encoder Section        ////////////////////////////////////////////////


/**
 * @brief Interrupt handles 
 * @note will update with direction reading later
 */
static void IRAM_ATTR left_encoder_interrupt_handler(void *args)
{
    uint32_t pinNumber = (uint32_t) args;
    switch (pinNumber)
    {
    case LEFT_ENCODER_A_PIN:
        if (gpio_get_level((gpio_num_t)LEFT_ENCODER_B_PIN))
        {   /* B is High when A triggered -> backwards? */
            /* rotating backward? */
        }
        break;
    case LEFT_ENCODER_B_PIN:
        if (gpio_get_level((gpio_num_t)LEFT_ENCODER_A_PIN))
        {   /* A is High when B trigger -> forwards? */
            /* rotating forward? */
        }
        
    default:
        break;
    }
    encoder_data.left_encoder_pulseCount++;  
}
static void IRAM_ATTR right_encoder_interrupt_handler(void *args)
{
    uint32_t pinNumber = (uint32_t) args;
    switch (pinNumber)
    {
    case RIGHT_ENCODER_A_PIN:
        if (gpio_get_level((gpio_num_t)RIGHT_ENCODER_B_PIN))
        {   /* B is High when A triggered -> backwards? */
            /* rotating backward? */
        }
        break;
    case RIGHT_ENCODER_B_PIN:
        if (gpio_get_level((gpio_num_t)RIGHT_ENCODER_A_PIN))
        {   /* A is High when B trigger -> forwards? */
            /* rotating forward? */
        }        
    default:
        break;
    } 
    encoder_data.right_encoder_pulseCount++;
}

/**
 * @brief Motor speed calculation
 */
float compute_motor_speed(uint32_t &count, float &dt)
{
    float ret = ((count / (float)ENCODER_PPR) / dt); /* Revolution per sec
                                                            assume dt is 10ms everytime */
    count = 0;
    return ret;                                 
}

/**
 * @brief init encoder
 */
void encoder_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / ENCODER_READ_FREQ_HZ);
    float dt = 1.0f / ENCODER_READ_FREQ_HZ;
    /* Encoder pins set to interrupt mode */
    gpio_config_t motor_io_conf =
    {
        /* bit mask of the pins, use GPIO16:19 */
        .pin_bit_mask = GPIO_ENCODER_PIN_SEL,
        /* set as input mode */
        .mode = GPIO_MODE_INPUT,
        /* enable pull-up mode */
        .pull_up_en = GPIO_PULLUP_ENABLE,
        /* disable pull-down resistor */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        /* interrupt of rising edge */
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&motor_io_conf);
    while (1)
    {
        if(xSemaphoreTake(mutex_encoder_data, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            encoder_data.left_motor_speed = compute_motor_speed(encoder_data.left_encoder_pulseCount, dt);
            encoder_data.right_motor_speed = compute_motor_speed(encoder_data.right_encoder_pulseCount, dt);
            xSemaphoreGive(mutex_encoder_data);
        }   
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}



/////////////////////////            Monitor Task           ////////////////////////



void monitor_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2000); // x ms second

    sensor_data_t monitor_sensor_data;
    // pid_controller_t monitor_speed_pid;
    // pid_controller_t monitor_roll_pid;
    control_data_t monitor_control_data;
    web_control_t monitor_web_control;
    while (1)
    {
        if (xSemaphoreTake(mutex_sensor_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            monitor_sensor_data = sensor_data;            
            xSemaphoreGive(mutex_sensor_data);
        }
        else if (xSemaphoreTake(mutex_control_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            monitor_control_data = control_data;
            xSemaphoreGive(mutex_control_data);
        }
        else if (xSemaphoreTake(mutex_web_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            monitor_web_control = web_control;
            xSemaphoreGive(mutex_web_data);
        }
        else
        {
            ESP_LOGW(TAG_MUTEX, "Failed to take mutex");
        }
        // printf("ROLL: %3.1f, Accel: %3.1f, speed PID Integral: %3.1f, PWM output: %3.1f \n", 
        //         monitor_sensor_data.roll, 
        //         monitor_sensor_data.accel_y, 
        //         monitor_speed_pid.integral, 
        //         monitor_control_data.left_motor_speed);

        // printf("Tilt setpoint: %3.1f° | Actual: %3.1f° | Error: %3.1f° | Output: %3.1f\n",
        //         monitor_roll_pid.setpoint,
        //         monitor_sensor_data.roll, 
        //         monitor_roll_pid.error,  // Error from desired setppint
        //         monitor_control_data.left_motor_speed);
        
        // printf("Tilt: %3.1f°\n",
        //         monitor_sensor_data.roll);

        // printf("motor rps: LEFT: %3.1f   RIGHT: %3.1f, Tilt angle: %3.1f\n", 
        //     monitor_control_data.left_motor_feedback,
        //     monitor_control_data.right_motor_feedback,
        //     monitor_sensor_data.roll
        // );

        // ESP_LOGI("BALANCE", "Web params - KP: %.2f, KI: %.2f, KD: %.2f, Target: %.2f, Enable: %d",
        // web_control.kp_roll, web_control.ki_roll, web_control.kd_roll, 
        // web_control.target_angle, web_control.enable_balance);

        printf("motor rps: LEFT: %3.1f   RIGHT: %3.1f\n", 
            encoder_data.left_motor_speed,
            encoder_data.right_motor_speed
        );

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    vTaskDelete(NULL);
}
/////////////////////////            MAIN APP SECTION       ////////////////////////

void app_main (void)
{
    /* create mutex keys */
    mutex_sensor_data = xSemaphoreCreateMutex();
    mutex_control_data = xSemaphoreCreateMutex();
    mutex_encoder_data = xSemaphoreCreateMutex();
    mutex_web_data = xSemaphoreCreateMutex();
    if (mutex_control_data == NULL || mutex_encoder_data == NULL || 
        mutex_sensor_data == NULL || mutex_web_data == NULL) 
    {
        ESP_LOGE("MAIN", "Failed to create mutex\n");
        return;
    }

    /* create a queue to handle GPIO event from isr */
    interrupt_queue = xQueueCreate(10, sizeof(uint32_t));
    /* install gpio isr service */
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    /* hook isr handler to specific encoder pin */
    gpio_isr_handler_add((gpio_num_t)LEFT_ENCODER_A_PIN, left_encoder_interrupt_handler, (void*) LEFT_ENCODER_A_PIN);
    gpio_isr_handler_add((gpio_num_t)LEFT_ENCODER_B_PIN, left_encoder_interrupt_handler, (void*) LEFT_ENCODER_B_PIN);
    gpio_isr_handler_add((gpio_num_t)RIGHT_ENCODER_A_PIN, right_encoder_interrupt_handler, (void*) RIGHT_ENCODER_A_PIN);
    gpio_isr_handler_add((gpio_num_t)RIGHT_ENCODER_B_PIN, right_encoder_interrupt_handler, (void*) RIGHT_ENCODER_B_PIN);

    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 1024*4, NULL, 5, NULL, 1);     /* Core 1: High priority for sensor readings */
    xTaskCreatePinnedToCore(balance_task, "balance_task", 1024*3, NULL, 4, NULL, 1);   /* Core 1: Critical control calculations */
    xTaskCreatePinnedToCore(encoder_task, "encoder_task", 1024*2, NULL, 3, NULL, 1);   /* Core 1: Encoder reading */
    xTaskCreatePinnedToCore(motor_task, "motor_task", 1024*2, NULL, 3, NULL, 1);       /* Core 1: Motor control updates  */
    xTaskCreatePinnedToCore(monitor_task, "monitor_task", 1024*4, NULL, 1, NULL, 1);   /* Core 1: Lowest priority for monitoring */
    xTaskCreatePinnedToCore(web_server_task, "web_server_task", WEB_SERVER_STACK_SIZE, NULL, 2, NULL, 1);

    ESP_LOGE("MAIN", "ALL tasks initilized succesfully");
}