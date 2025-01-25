#include "camera.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "ov5642_regs.h"


// SPI configuration
#define CS_PIN 5         // Chip Select GPIO
#define SCLK_PIN 18      // SPI Clock GPIO
#define MOSI_PIN 23      // Master Out Slave In GPIO
#define MISO_PIN 19      // Master In Slave Out GPIO
#define TAG "CameraSetup" // Logging tag

// I2C camera address
#define CAMERA_I2C_ADDR 0x3C

#define I2C_MASTER_NUM I2C_NUM_0      // I2C port number


void write_i2c(uint16_t reg, uint8_t value) {
    uint8_t data[3] = {(reg >> 8) & 0xFF, reg & 0xFF, value};
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, CAMERA_I2C_ADDR, data, sizeof(data), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to register 0x%04X: %s", reg, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Wrote 0x%02X to register 0x%04X", value, reg);
    }
}

uint8_t read_i2c(uint16_t reg) {
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

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SPI initialized successfully.");
}

void setup_camera_I2C(void) {
    write_i2c(0x3008, 0x82);  // Software reset
    vTaskDelay(pdMS_TO_TICKS(200));

    write_i2c(0x3103, 0x93);  // Clock selection
    write_i2c(0x3011, 0x08);  // PLL settings for 24MHz
    write_i2c(0x3008, 0x42);  // Set power mode
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set resolution (VGA)
    write_i2c(0x3808, 0x02);  // VGA width (high byte)
    write_i2c(0x3809, 0x80);  // VGA width (low byte)
    write_i2c(0x380A, 0x01);  // VGA height (high byte)
    write_i2c(0x380B, 0xE0);  // VGA height (low byte)

    // Enable test pattern
    write_i2c(0x503D, 0x80);  // Enable color bar test pattern
    write_i2c(0x503e, 0x00);

    // Start streaming
    write_i2c(0x3008, 0x02);  // Start streaming
    vTaskDelay(pdMS_TO_TICKS(200));


    /*
    // Confirm sensor is detected
    uint8_t vid = read_i2c(0x300A);  // Read Vendor ID
    uint8_t pid = read_i2c(0x300B);  // Read Product ID
    if (vid == 0x56 && pid == 0x42) {
        ESP_LOGI(TAG, "OV5642 detected. VID: 0x%02X, PID: 0x%02X", vid, pid);
    } else {
        ESP_LOGE(TAG, "OV5642 not detected. VID: 0x%02X, PID: 0x%02X", vid, pid);
    }
    */
}

void captureImage(void) {
    // Configure FIFO
    write_i2c(0x4202, 0x00);  // Disable FIFO
    vTaskDelay(pdMS_TO_TICKS(10));
    write_i2c(0x4202, 0x04);  // Clear FIFO
    vTaskDelay(pdMS_TO_TICKS(10));
    write_i2c(0x4202, 0x01);  // Enable FIFO
    vTaskDelay(pdMS_TO_TICKS(100));  // Allow FIFO to populate

    // Wait for FIFO to fill
    uint8_t fifo_status = 0;
    for (int i = 0; i < 10; i++) {
        fifo_status = read_i2c(0x4202);
        if (fifo_status & 0x08) {  // Check FIFO_READY bit
            ESP_LOGI(TAG, "Capture completed.");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!(fifo_status & 0x08)) {
        ESP_LOGE(TAG, "Capture failed: FIFO not ready.");
        //return;
    }

    // Read FIFO size
    uint8_t size_high = read_i2c(0x4203);
    uint8_t size_mid = read_i2c(0x4204);
    uint8_t size_low = read_i2c(0x4205);
    uint32_t fifo_size = (size_high << 16) | (size_mid << 8) | size_low;
    ESP_LOGI(TAG, "Captured image size: %u bytes", (unsigned int) fifo_size);

    // Read image data from FIFO via SPI
    uint8_t buffer[256];  // Buffer for SPI reads
    uint32_t bytes_read = 0;

    while (bytes_read < fifo_size) {
        uint32_t chunk_size = (fifo_size - bytes_read > sizeof(buffer)) ? sizeof(buffer) : (fifo_size - bytes_read);
        spi_transaction_t trans = {
            .length = chunk_size * 8, // Length in bits
            .rx_buffer = buffer
        };
        esp_err_t ret = spi_device_transmit(spi, &trans);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Read %u bytes from FIFO.", (unsigned int) chunk_size);
            bytes_read += chunk_size;
        } else {
            ESP_LOGE(TAG, "SPI read error: %s", esp_err_to_name(ret));
            break;
        }
    }
}


void setupCamera(void) {    
    setup_camera_I2C();

    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for 100ms after powering the shield
    
    setupSPI();
}
