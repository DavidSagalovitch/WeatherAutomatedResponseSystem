#include "../motor/motor.h"  // Motor control functions
#include "../led/led.h"    // LED blink functions
#include "../testIO/gpio_read.h"     // GPIO read functionality for switch
#include "../testIO/gpio_write.h" // GPIO write functionality for switch
#include "../sensors/sensors_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    // Start the motor task
    xTaskCreate(motor_task, "Motor Task", 4096, NULL, 1, NULL);

    // Start the LED blinking task
    xTaskCreate(led_task, "LED Blink Task", 2048, NULL, 1, NULL);

    // Start the GPIO read task (to monitor the switch)
    xTaskCreate(gpio_read_task, "GPIO Read Task", 2048, NULL, 1, NULL);

    // Start the GPIO write task (to simulate the switch output)
    xTaskCreate(gpio_write_task, "GPIO Write Task", 2048, NULL, 1, NULL);

    xTaskCreate(sensors_run, "Sensors Task", 8172, NULL, 1, NULL);

}
