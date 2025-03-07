#include "I2Csetup.h"
#include "driver/i2c.h"
#include "esp_log.h"

// I2C configuration
#define I2C_MASTER_SCL_IO 22          // GPIO number for I2C SCL
#define I2C_MASTER_SDA_IO 21          // GPIO number for I2C SDA
#define I2C_MASTER_NUM I2C_NUM_0      // I2C port number
#define I2C_MASTER_FREQ_HZ 100000
#define CAMERA_I2C_ADDR 0x3C          // OV5642 default I2C address
#define LIDAR_I2C_ADDR 0x40           // LiDAR default I2C address
#define TAG "I2CSetup"


void setupI2C(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "I2C driver installed successfully");
    }
}

void sendI2CCommand(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, CAMERA_I2C_ADDR, data, sizeof(data), pdMS_TO_TICKS(1000));

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C command sent: reg=0x%02X, value=0x%02X", reg, value);
    } else {
        ESP_LOGE(TAG, "Failed to send I2C command: %s", esp_err_to_name(ret));
    }
}

void i2cScanner(void) {
<<<<<<< HEAD
    const int target_addrs[] = {0x3C}; // Target addresses to find
    const size_t num_targets = sizeof(target_addrs) / sizeof(target_addrs[0]);
    const int timeout_ms = 100;           // Timeout in milliseconds
    const int scan_interval_ms = 500;      // Time between scans
    int elapsed_time = 0;

    ESP_LOGI(TAG, "Starting I2C bus scan...");

    while (elapsed_time < timeout_ms) {
        bool all_found = true;

        // Scan the entire I2C bus and list detected devices
        ESP_LOGI(TAG, "Scanning I2C bus...");
        for (int addr = 1; addr < 127; addr++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Found I2C device at address: 0x%02X", addr);
            }
        }
        ESP_LOGI(TAG, "I2C scan complete.");

        // Check if all target addresses are found
        for (size_t i = 0; i < num_targets; i++) {
            int target_addr = target_addrs[i];
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (target_addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
            i2c_cmd_link_delete(cmd);

            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Target I2C device not found at address: 0x%02X", target_addr);
                all_found = false;
            } else {
                ESP_LOGI(TAG, "Target I2C device found at address: 0x%02X", target_addr);
            }
        }

        // If all target addresses are found, exit the loop
        if (all_found) {
            ESP_LOGI(TAG, "All target I2C devices found. Exiting scan.");
            return;
        }

        // Delay and increment elapsed time
        vTaskDelay(pdMS_TO_TICKS(scan_interval_ms));
        elapsed_time += scan_interval_ms;
    }

    ESP_LOGE(TAG, "Timeout reached. Not all target I2C devices were found.");
=======
    ESP_LOGI(TAG, "Scanning I2C Bus...");
    int devices = 0;
    for (int addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        ESP_LOGI(TAG, "Testing Address: 0x%02X", addr);
        if (ret == ESP_OK) {
            devices++;
            ESP_LOGI(TAG, "Found I2C device at address: 0x%02X", addr);
        } 
    }
    if (devices == 0) {
        ESP_LOGI(TAG, "NO I2C DEVICE FOUND");
    } else {
        ESP_LOGI(TAG, "%i I2C DEVICES FOUND", devices);
    }
    ESP_LOGI(TAG, "I2C Scan Complete.");
>>>>>>> cc236440ee9d56500149a8fc04635edf0b806571
}
