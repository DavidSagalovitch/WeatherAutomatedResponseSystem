#include <stdio.h>
#include "../i2c/I2Csetup.h"
#include "camera.h"
#include "lidar.h"
#include "sensors_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"


void sensors_run(void *pvParameters) {

    setupI2C();

    vTaskDelay(100);

    i2cScanner();

<<<<<<< HEAD
    vTaskDelay(100);

    setupCamera();

=======
    // Initialize SPI and set up the camera
    //setupCamera();
    //float distance;
>>>>>>> cc236440ee9d56500149a8fc04635edf0b806571
    while (1) {
      /*
      captureImage();
      distance = read_distance();
      if (distance >= 0) {
          printf("Measured Distance: %.2f cm\n", distance);
      }
      vTaskDelay(pdMS_TO_TICKS(500));  // Wait for 500ms
      */
      if (detect_water()) {
        printf("Water detected on windshield!\n");
      } else {
        printf("Windshield is dry.\n");
      }
      vTaskDelay(pdMS_TO_TICKS(100));  // Check every half second
    }
}
