#include "gpio_write.h"
#include <stdio.h>
#include "driver/gpio.h"

void gpio_write_task(void *pvParameters)
{
    // Configure GPIO 27 as output
    gpio_reset_pin(GPIO_OUTPUT_PIN);
    gpio_set_direction(GPIO_OUTPUT_PIN, GPIO_MODE_OUTPUT);

    static bool state = true;
    gpio_set_level(GPIO_OUTPUT_PIN, state);

    printf("GPIO 27 set to %d\n", state);

    vTaskDelete(NULL); 
}
