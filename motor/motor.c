#include "motor.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../sensors/sensors_main.h"

void motor_init(void)
{
    // Configure the motor direction GPIO
    gpio_reset_pin(MOTOR_DIR);
    gpio_set_direction(MOTOR_DIR, GPIO_MODE_OUTPUT);

    // Configure the LEDC PWM timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure the LEDC PWM channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = MOTOR_PWM,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,  // Initial duty cycle
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void motor_spin_forward(int speed)
{
    gpio_set_level(MOTOR_DIR, 1);  // Set direction to forward
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, speed);  // Set duty cycle
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    printf("Motor spinning forward at speed %d\n", speed);
}

void motor_spin_reverse(int speed)
{
    gpio_set_level(MOTOR_DIR, 0);  // Set direction to reverse
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, speed);  // Set duty cycle
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    printf("Motor spinning reverse at speed %d\n", speed);
}

void motor_stop(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);  // Stop motor
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    printf("Motor stopped\n");
}

void motor_task(void *pvParameters)
{
    motor_init();
    while (1) {
        if (rain_detected){
            if (whiper_speed_ms < 116) {
                whiper_speed_ms = 116; // Prevent overspeeding
            }

            int speed = (256 * 116) / whiper_speed_ms; // Scale speed based on desired time

            // Ensure speed does not exceed maximum limit
            if (speed > 256) {
                speed = 256;
            }

            printf("Setting motor speed to %d for 180-degree rotation in %d ms\n", speed, whiper_speed_ms);

            // Spin left (reverse) for 180°
            motor_spin_reverse(speed);
            vTaskDelay(pdMS_TO_TICKS(whiper_speed_ms));
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(500)); // Small delay before reversing

            // Spin right (forward) for 180°
            motor_spin_forward(speed);
            vTaskDelay(pdMS_TO_TICKS(whiper_speed_ms));
            motor_stop();
        }
    }
}
