#include <stdio.h>
#include "../i2c/I2Csetup.h"
#include "camera.h"
#include "sensors_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void sensors_run(void *pvParameters) {
    // Initialize I2C for camera configuration
    setupI2C();

    i2cScanner();

    // Initialize SPI and set up the camera
    setupCamera();

   // while (1) {
   //     // Capture an image every 10 seconds
   //     captureImage();
  //      vTaskDelay(10000 / portTICK_PERIOD_MS);
  //  }
      vTaskDelete(NULL); 

}
