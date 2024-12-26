#include "gpio_read.h"
#include <stdio.h>
#include "driver/gpio.h"

// Shared variable
volatile bool turn_on = false;

void gpio_read_task(void *pvParameters)
{
    // Configure GPIO 12 as input
    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << GPIO_INPUT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&input_conf);

    while (1)
    {
        // Read GPIO 12 and update the shared variable
        int pin_state = gpio_get_level(GPIO_INPUT_PIN);
        if (pin_state == 1)
        {
            turn_on = true;
            printf("GPIO 12 is HIGH. Setting turn_on to true.\n");
        }
        else
        {
            turn_on = false;
            printf("GPIO 12 is LOW. Setting turn_on to false.\n");
        }

        // Delay for stability
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
