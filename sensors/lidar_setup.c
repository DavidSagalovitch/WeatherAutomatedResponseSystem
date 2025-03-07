#include "lidar.h"
#include <math.h>
#include <stdbool.h>
#include "esp_log.h"
#include "driver/i2c.h"

// I2C lidar address
#define LIDAR_I2C_ADDR 0x40
#define REG_DISTANCE_UPPER 0x5E
#define REG_DISTANCE_LOWER 0x5F
#define REG_SHIFT 0x35

#define SAMPLE_COUNT 10
#define WATER_THRESHOLD 0.5

#define TAG "LidarSetup"

#define I2C_MASTER_NUM I2C_NUM_0      // I2C port number

esp_err_t i2c_read_register(uint8_t reg_addr, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // First cycle: Send register address (Write Mode)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIDAR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;  // Exit if error in first cycle
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Short delay for device readiness

    // Second cycle: Read data (Read Mode)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIDAR_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);  // Read single byte
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

float read_distance() {
    uint8_t dist_upper, dist_lower, shift;

    if (i2c_read_register(0x5E, &dist_upper) != ESP_OK) {
        printf("Error reading upper distance data.\n");
        return -1;
    }
    if (i2c_read_register(0x5F, &dist_lower) != ESP_OK) {
        printf("Error reading lower distance data.\n");
        return -1;
    }
    if (i2c_read_register(0x35, &shift) != ESP_OK) {
        printf("Error reading shift coefficient.\n");
        return -1;
    }

    // Combine 12-bit raw distance value
    uint16_t distance_raw = ((uint16_t)dist_upper << 4) | (dist_lower & 0x0F);
    
    // Convert to cm: distance_cm = Distance[11:0] / (16 * (2^n))
    float distance_cm = (float)distance_raw / 16.0 / (1 << shift);

    return distance_cm;
}

bool detect_water() {
    float distances[SAMPLE_COUNT];
    float sum = 0, mean, variance = 0, stddev;

    // Collect multiple distance readings
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        distances[i] = read_distance();
        //printf("Measured Distance: %.2f cm\n", distances[i]);
        sum += distances[i];
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between readings
    }

    mean = sum / SAMPLE_COUNT;

    // Compute variance
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        variance += pow(distances[i] - mean, 2);
    }
    variance /= SAMPLE_COUNT;
        stddev = sqrt(variance);
    printf("STD DEV: %.2f\n", stddev);
    // Check if variance exceeds threshold
    if (stddev > WATER_THRESHOLD) {
        return true;  // Water detected
    }
    return false;  // Dry windshield
}