#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// PWM and GPIO definitions for motor
#define MOTOR_PWM   16  // Motor PWM pin
#define MOTOR_DIR   17  // Motor direction pin

// PWM Configuration
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define LEDC_FREQUENCY  5000              // Frequency in Hz (5kHz)

void motor_task(void *pvParameters);
void motor_init(void);
void motor_spin_forward(int speed);
void motor_spin_reverse(int speed);
void motor_stop(void);

#endif // MOTOR_CONTROL_H