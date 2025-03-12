#include "camera.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "ov5642_regs.h"
#include "../image_processing/processImage.h"


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
    uint8_t reg_addr[2] = {(reg >> 8) & 0xFF, reg & 0xFF};  // Register address (high byte, low byte)
    uint8_t value = 0;  // Variable to store the read value

    // Send register address (write phase)
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, CAMERA_I2C_ADDR, reg_addr, sizeof(reg_addr), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address 0x%04X: %s", reg, esp_err_to_name(ret));
        return 0;
    }

    // Read the value from the register
    ret = i2c_master_read_from_device(I2C_MASTER_NUM, CAMERA_I2C_ADDR, &value, 1, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from register 0x%04X: %s", reg, esp_err_to_name(ret));
        return 0;
    }

    ESP_LOGI(TAG, "Read 0x%02X from register 0x%04X", value, reg);
    return value;
}


void setup_camera_I2C(void) {
    ESP_LOGI(TAG, "Initializing OV5642 camera for 650x480 RAW output...");

    // Reset sensor
    write_i2c(0x3008, 0x80);  
    vTaskDelay(pdMS_TO_TICKS(200));

    // Set sensor to RAW mode
    write_i2c(0x3818, 0x81); // Enable RAW output

    // Configure resolution to 650x480
    for (int i = 0; OV5642_640x480_RAW[i].reg != 0xFFFF; i++) {
        write_i2c(OV5642_640x480_RAW[i].reg, OV5642_640x480_RAW[i].val);
        ESP_LOGI(TAG, "Wrote 0x%02X to register 0x%04X", OV5642_640x480_RAW[i].val, OV5642_640x480_RAW[i].reg);
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for stability
    }

    // Start streaming
    write_i2c(0x3008, 0x02);  // Power on and start streaming
    vTaskDelay(pdMS_TO_TICKS(200));

    // Confirm sensor is detected
    uint8_t vid = read_i2c(0x300A);
    uint8_t pid = read_i2c(0x300B);
    if (vid == 0x56 && pid == 0x42) {
        ESP_LOGI(TAG, "OV5642 detected. VID: 0x%02X, PID: 0x%02X", vid, pid);
    } else {
        ESP_LOGE(TAG, "OV5642 not detected. VID: 0x%02X, PID: 0x%02X", vid, pid);
    }
}

// SPI device handle
spi_device_handle_t spi;

// SPI Write Helper
void write_spi(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {0x80 | reg, value};  // Set Bit[7] for write operation
    spi_transaction_t trans = {
        .length = 16,       // Total bits to transfer (2 bytes)
        .tx_buffer = data,  // Pointer to data to transmit
        .rx_buffer = NULL,  // No data expected to be received
    };

    esp_err_t ret = spi_device_transmit(spi, &trans);  // Perform SPI transaction
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write SPI register 0x%02X: %s", reg, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Successfully wrote 0x%02X to SPI register 0x%02X", value, reg);
    }
}

uint8_t read_spi(uint8_t reg) {
    uint8_t tx_data[2] = {reg & 0x7F, 0x00};  // Clear Bit[7] for read operation; second byte is dummy
    uint8_t rx_data[2] = {0x00, 0x00};       // Buffer to store received data

    spi_transaction_t trans = {
        .length = 16,        // Total bits to transfer (2 bytes)
        .tx_buffer = tx_data, // Pointer to data to transmit
        .rx_buffer = rx_data, // Pointer to buffer to receive data
    };

    esp_err_t ret = spi_device_transmit(spi, &trans);  // Perform SPI transaction
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SPI register 0x%02X: %s", reg, esp_err_to_name(ret));
        return 0;
    } else {
        ESP_LOGI(TAG, "Successfully read 0x%02X from SPI register 0x%02X", rx_data[1], reg);
    }

    return rx_data[1];  // The second byte contains the received data
}

void setupSPI(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4 * 1000 * 1000,  // Set to 4 MHz
        .mode = 0,  
        .spics_io_num = CS_PIN,
        .queue_size = 7
    };

    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SPI initialized successfully.");
    
    // Configure CPLD for VSYNC handling
    write_spi(0x07, 0x80);  // Reset CPLD
    vTaskDelay(pdMS_TO_TICKS(100));
    write_spi(0x07, 0x00);  // Exit reset
    vTaskDelay(pdMS_TO_TICKS(100));
    write_spi(0x03, read_spi(0x03) | 0x04);  // Set VSYNC active HIGH
}

void debugSPI(void) {
    uint8_t value;

    // Test writing and reading a known working register
    write_spi(0x00, 0x55);
    value = read_spi(0x00);
    ESP_LOGI(TAG, "Test Register 0x00: 0x%02X", value);

    // Test writing and reading FIFO Control Register
    write_spi(0x04, 0x20);
    vTaskDelay(pdMS_TO_TICKS(10));
    value = read_spi(0x04);
    ESP_LOGI(TAG, "FIFO Control Register (0x04) after clearing: 0x%02X", value);

    write_spi(0x04, 0x10);
    vTaskDelay(pdMS_TO_TICKS(10));
    value = read_spi(0x04);
    ESP_LOGI(TAG, "FIFO Control Register (0x04) after write pointer reset: 0x%02X", value);

    write_spi(0x04, 0x08);
    vTaskDelay(pdMS_TO_TICKS(10));
    value = read_spi(0x04);
    ESP_LOGI(TAG, "FIFO Control Register (0x04) after read pointer reset: 0x%02X", value);

    write_spi(0x04, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    value = read_spi(0x04);
    ESP_LOGI(TAG, "FIFO Control Register (0x04) after enabling: 0x%02X", value);

    // Test reading FIFO Status Register
    value = read_spi(0x41);
    ESP_LOGI(TAG, "FIFO Status Register (0x41): 0x%02X", value);
}

void reset_camera_via_spi(void) {
    ESP_LOGI(TAG, "Starting SPI reset sequence for the camera...");

    write_spi(0x04, 0x10);  // Reset FIFO write pointer
    write_spi(0x04, 0x08);  // Reset FIFO read pointer
    write_spi(0x04, 0x01);  // Clear FIFO write done flag

    write_spi(0x05, 0x07);  // Set GPIOs as output

    write_spi(0x03, 0x00);  // Disable FIFO mode, default Hsync/Vsync

    write_spi(0x01, 0x00);  // Stop capture

    write_spi(0x04, 0x20);  // Clear power mode

    write_spi(0x06, 0x01);  // Assert sensor reset
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Camera reset sequence via SPI completed.");
}

float captureImage(void) {
    ESP_LOGI(TAG, "Starting image capture...");

    // Reset and start sensor
    write_spi(0x06, 0x01);  // Assert sensor reset
    vTaskDelay(pdMS_TO_TICKS(200));
    write_spi(0x06, 0x00);  // Deassert sensor reset
    vTaskDelay(pdMS_TO_TICKS(200));

    // Enable FIFO mode
    write_spi(0x05, 0x07);
    write_spi(0x03, 0x10);

    // Reset FIFO
    write_spi(0x04, 0x20);
    write_spi(0x04, 0x10);
    write_spi(0x04, 0x08);
    write_spi(0x04, 0x01);

    // Start capture
    write_spi(0x01, 0x01);
    write_spi(0x04, 0x02);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Wait for capture completion
    uint8_t status = read_spi(0x41);
    if (!(status & 0x08)) {
        ESP_LOGE(TAG, "Capture failed: FIFO not ready.");
        return 0;
    }

    // Read FIFO size
    uint32_t size = (read_spi(0x44) << 16) | (read_spi(0x43) << 8) | read_spi(0x42);
    ESP_LOGI(TAG, "Captured image size: %u bytes",(unsigned int)size);

    if (size < 1000) {  
        ESP_LOGE(TAG, "Invalid image size. Capture may have failed.");
        return 0;
    }

    // Allocate memory for image
    uint8_t *image_data = (uint8_t *)malloc(size);
    if (!image_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for image.");
        return 0;
    }

    // Read FIFO data into buffer
    uint32_t bytes_read = 0;
    while (bytes_read < size) {
        uint32_t chunk = (size - bytes_read > 256) ? 256 : (size - bytes_read);
        spi_transaction_t trans = {
            .length = chunk * 8,
            .rx_buffer = &image_data[bytes_read]
        };

        esp_err_t ret = spi_device_transmit(spi, &trans);
        if (ret == ESP_OK) {
            bytes_read += chunk;
        } else {
            ESP_LOGE(TAG, "SPI read error: %s", esp_err_to_name(ret));
            free(image_data);
            return 0;
        }
    }

    ESP_LOGI(TAG, "Successfully read %u bytes from FIFO.",(unsigned int)bytes_read);

    //processimage
    float rain_intensity = process_image(image_data);

    // Free buffer
    free(image_data);
    
    reset_camera_via_spi();

    return rain_intensity;
}



void setupCamera(void) {   
    setupSPI();
    vTaskDelay(pdMS_TO_TICKS(100)); 
    setup_camera_I2C();
    vTaskDelay(pdMS_TO_TICKS(1000)); 

}
