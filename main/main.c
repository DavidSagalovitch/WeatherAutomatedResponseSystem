#include "../motor/motor.h"  // Motor control functions
#include "../led/led.h"    // LED blink functions
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    // Initialize the motor
    motor_init();

    // Start the LED blinking task
    xTaskCreate(led_blink, "LED Blink Task", 2048, NULL, 1, NULL);

    while (1) {
        // Spin motor forward for 2 seconds
        motor_spin_forward(256);  // Slow speed
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Spin motor reverse for 2 seconds
        motor_spin_reverse(256);  // Slow speed
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Stop motor for 1 second
        motor_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
