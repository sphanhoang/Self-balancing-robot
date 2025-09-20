#include "balance_robot.h"

extern "C" {
	void app_main(void);
}

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
        .frequency = 200,         /* 10k Hz to reduce noise */
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
 * @note Pretty self-explanatory isn't it?
 */
esp_err_t motor_control(float *speed)
{
    float abs_speed = *speed;
    
    if (*speed < 0)
    {
        gpio_set_level((gpio_num_t)MOTOR_IN1_PIN, 0);
        gpio_set_level((gpio_num_t)MOTOR_IN2_PIN, 1);
        gpio_set_level((gpio_num_t)MOTOR_IN3_PIN, 1);
        gpio_set_level((gpio_num_t)MOTOR_IN4_PIN, 0);
        abs_speed = -*speed;
    }
    else
    {
        gpio_set_level((gpio_num_t)MOTOR_IN1_PIN, 1);
        gpio_set_level((gpio_num_t)MOTOR_IN2_PIN, 0);
        gpio_set_level((gpio_num_t)MOTOR_IN3_PIN, 0);
        gpio_set_level((gpio_num_t)MOTOR_IN4_PIN, 1);
    }
    (abs_speed > 255.0f) ? abs_speed = 255.0f : abs_speed = abs_speed;
    (abs_speed < 38.25f) ? abs_speed = 38.2f : abs_speed = abs_speed;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (abs_speed/255.0f)*100.0f); /* Druve Motor A */
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, (abs_speed/255.0f)*100.0f); /* Drive Motor B */
    return ESP_OK;
}

/**
 * @brief Task to update motor control at a fixed frequency
 */
void motor_task (void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / MOTOR_UPDATE_FREQ_HZ);
    motor_init();

    
    float speed = 50; // Test speed value
    while (1)
    {
        motor_control(&speed);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void app_main (void)
{
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 10, NULL);
}