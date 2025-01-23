#include "camera.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/i2c.h"


// SPI configuration
#define CS_PIN 5         // Chip Select GPIO
#define SCLK_PIN 18      // SPI Clock GPIO
#define MOSI_PIN 23      // Master Out Slave In GPIO
#define MISO_PIN 19      // Master In Slave Out GPIO
#define TAG "CameraSetup" // Logging tag

// I2C camera address
#define CAMERA_I2C_ADDR 0x3C

#define I2C_MASTER_NUM I2C_NUM_0      // I2C port number

void i2c_write_register(uint16_t reg, uint8_t value) {
    uint8_t data[3] = {(reg >> 8) & 0xFF, reg & 0xFF, value};
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, CAMERA_I2C_ADDR, data, sizeof(data), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to register 0x%04X: %s", reg, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Wrote 0x%02X to register 0x%04X", value, reg);
    }
}

uint8_t i2c_read_register(uint16_t reg) {
    uint8_t reg_addr[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
    uint8_t value = 0;

    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, CAMERA_I2C_ADDR, reg_addr, sizeof(reg_addr), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address 0x%04X: %s", reg, esp_err_to_name(ret));
        return 0;
    }

    ret = i2c_master_read_from_device(I2C_MASTER_NUM, CAMERA_I2C_ADDR, &value, 1, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from register 0x%04X: %s", reg, esp_err_to_name(ret));
        return 0;
    }

    ESP_LOGI(TAG, "Read 0x%02X from register 0x%04X", value, reg);
    return value;
}

void setup_camera_I2C(void) {
    i2c_write_register(0x3008, 0x82);  // Software reset
    vTaskDelay(pdMS_TO_TICKS(200));    // Wait for reset to complete

    i2c_write_register(0x3b07, 0x09);  // Enable I2C master mode
    i2c_write_register(0x3016, 0x02);  // GPIO configuration
    i2c_write_register(0x3017, 0xFF);  // Freerun enable
    i2c_write_register(0x3709, 0x10);  // Sensor control
    i2c_write_register(0x3b04, 0x04);  // I2C clock setting
    i2c_write_register(0x3b05, 0x00);  // Reset I2C operation

    uint8_t init_sequence[][2] = {
        {0x3103, 0x11},  // System clock from PLL
        {0x3008, 0x42},  // Power management
        {0x3017, 0x7F},  // Freerun enable
        {0x3018, 0xFC},  // Freerun enable
        {0x3810, 0xC2},  // Timing settings
        {0x3615, 0xF0},  // Array control
        {0x4300, 0x30},  // YUV output format
    };

    for (int i = 0; i < sizeof(init_sequence) / sizeof(init_sequence[0]); i++) {
        i2c_write_register(init_sequence[i][0], init_sequence[i][1]);
        vTaskDelay(pdMS_TO_TICKS(10));  // Delay between commands
    }

    uint8_t vid = i2c_read_register(0x300A);  // Read Vendor ID
    uint8_t pid = i2c_read_register(0x300B);  // Read Product ID

    if (vid == 0x56 && pid == 0x42) {
        ESP_LOGI(TAG, "Camera OV5642 detected successfully. VID: 0x%02X, PID: 0x%02X", vid, pid);
    } else {
        ESP_LOGE(TAG, "Failed to detect camera OV5642. VID: 0x%02X, PID: 0x%02X", vid, pid);
    }
}

spi_device_handle_t spi;  // SPI device handle

void setupSPI(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCLK_PIN,
        .quadwp_io_num = -1,  // Not used
        .quadhd_io_num = -1,  // Not used
        .max_transfer_sz = 4096  // Maximum transfer size in bytes
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5 * 1000 * 1000,  // 5 MHz
        .mode = 0,  // SPI mode 0
        .spics_io_num = CS_PIN,  // Chip select pin
        .queue_size = 7          // Transaction queue size
    };

    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    // Add the camera device to the SPI bus
    spi_device_handle_t spi;
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI initialized successfully.");
}

void setupCamera(void) {

    
    setup_camera_I2C();

    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for 100ms after powering the shield
    
    setupSPI();
}

void captureImage(void) {
    uint8_t command = 0x00;  // Replace with actual SPI command to capture an image
    uint8_t response;

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &command,
        .rx_buffer = &response,
    };

    // Start image capture
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));

    ESP_LOGI(TAG, "Capture command sent to camera.");

    // Poll for capture completion (example logic)
    command = 0x41;  // Example register for capture status
    do {
        spi_device_transmit(spi, &t);
    } while (!(response & 0x08));  // Check capture complete bit

    ESP_LOGI(TAG, "Image capture complete.");

    // Implement logic to read the image data via SPI
}