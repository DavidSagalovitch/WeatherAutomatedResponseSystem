#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Define GPIO pin for LED
#define LED_GPIO 2  // D2 corresponds to GPIO2

void app_main(void)
{
    // Configure GPIO pin as output
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Quick blink: 200ms ON, 200ms OFF
        gpio_set_level(LED_GPIO, 1);  // Turn LED ON
        printf("Quick Blink ON\n");
        vTaskDelay(200 / portTICK_PERIOD_MS);  // Wait 200ms
        gpio_set_level(LED_GPIO, 0);  // Turn LED OFF
        printf("Quick Blink OFF\n");
        vTaskDelay(200 / portTICK_PERIOD_MS);  // Wait 200ms

        // Long blink: 1 second ON, 1 second OFF
        gpio_set_level(LED_GPIO, 1);  // Turn LED ON
        printf("Long Blink ON\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait 1 second
        gpio_set_level(LED_GPIO, 0);  // Turn LED OFF
        printf("Long Blink OFF\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait 1 second
    }
}
