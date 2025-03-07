#include "led.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// GPIO definition for LED
#define LED_GPIO 2  // LED connected to GPIO2

void led_task(void *pvParameters)
{
    // Configure LED GPIO as output
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Quick blink: 200ms ON, 200ms OFF
        gpio_set_level(LED_GPIO, 1);
        printf("Quick Blink ON\n");
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_GPIO, 0);
        printf("Quick Blink OFF\n");
        vTaskDelay(pdMS_TO_TICKS(200));

        // Long blink: 1 second ON, 1 second OFF
        gpio_set_level(LED_GPIO, 1);
        printf("Long Blink ON\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_set_level(LED_GPIO, 0);
        printf("Long Blink OFF\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
