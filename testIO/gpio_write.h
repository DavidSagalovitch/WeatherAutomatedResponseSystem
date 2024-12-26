#ifndef GPIO_WRITE_H
#define GPIO_WRITE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPIO_OUTPUT_PIN 27  // GPIO for output

// Task function
void gpio_write_task(void *pvParameters);

#endif // GPIO_WRITE_H
