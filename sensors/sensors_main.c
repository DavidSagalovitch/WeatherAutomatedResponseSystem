#include <stdio.h>
#include "../i2c/I2Csetup.h"
#include "camera.h"
#include "sensors_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"


void sensors_run(void *pvParameters) {

    setupI2C();

    vTaskDelay(100);

    i2cScanner();

    vTaskDelay(100);

    setupCamera();

    while (1) {
       //Capture an image every 10 seconds
        captureImage();
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
      vTaskDelete(NULL); 

}
