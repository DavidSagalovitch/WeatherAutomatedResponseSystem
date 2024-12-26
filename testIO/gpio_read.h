#ifndef GPIO_READ_H
#define GPIO_READ_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPIO_INPUT_PIN 12  // GPIO for input

// Shared variable
extern volatile bool turn_on;

// Task function
void gpio_read_task(void *pvParameters);

#endif // GPIO_READ_H
