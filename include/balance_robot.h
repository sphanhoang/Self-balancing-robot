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

#define MOTOR_UPDATE_FREQ_HZ 100  // 100Hz motor update
