#include <stdio.h>
#include "../i2c/I2Csetup.h"
#include "camera.h"
#include "lidar.h"
#include "sensors_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

volatile uint16_t whiper_speed_ms = 0;
bool rain_detected = false;

void sensors_run(void *pvParameters) {
    setupI2C();

    vTaskDelay(100);

    i2cScanner();

    vTaskDelay(100);

    // Initialize SPI and set up the camera
    setupCamera();

    //float distance;
    while (1) {
      captureImage();

      /*
      distance = read_distance();
      if (distance >= 0) {
          printf("Measured Distance: %.2f cm\n", distance);
      }
      vTaskDelay(pdMS_TO_TICKS(500));  // Wait for 500ms
      */
      if (detect_water()) {
        printf("Water detected on windshield!\n");
        rain_detected = true;
        whiper_speed_ms = 1000;
      } else {
        printf("Windshield is dry.\n");
        rain_detected = false;
        whiper_speed_ms = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(100));  // Check every half second
    }
}
