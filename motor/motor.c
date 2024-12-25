#include "motor.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

// PWM and GPIO definitions for motor
#define MOTOR_PWM   16  // Motor PWM pin
#define MOTOR_DIR   17  // Motor direction pin

// PWM Configuration
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define LEDC_FREQUENCY  5000              // Frequency in Hz (5kHz)

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