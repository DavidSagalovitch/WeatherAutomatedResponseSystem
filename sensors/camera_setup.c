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

void setup_camera_I2C(void) {
    // Software reset
    write_i2c(0x3008, 0x82);
    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for reset to complete

    // Clock and power settings
    write_i2c(0x3103, 0x93);  // Clock selection (external oscillator)
    write_i2c(0x3011, 0x08);  // PLL settings for 24MHz input clock
    write_i2c(0x3008, 0x42);  // Set power mode to normal
    vTaskDelay(pdMS_TO_TICKS(100));  // Allow power mode to stabilize

    // Set resolution to VGA (640x480)
    write_i2c(0x3808, 0x02);  // VGA width high byte (640)
    write_i2c(0x3809, 0x80);  // VGA width low byte
    write_i2c(0x380A, 0x01);  // VGA height high byte (480)
    write_i2c(0x380B, 0xE0);  // VGA height low byte

    // Enable test pattern (optional for debugging purposes)
    write_i2c(0x503D, 0x80);  // Enable color bar test pattern
    write_i2c(0x503E, 0x00);  // Ensure proper test pattern setup (if required)

    // Start streaming to activate the sensor
    write_i2c(0x3008, 0x02);  // Start sensor streaming
    vTaskDelay(pdMS_TO_TICKS(200));  // Allow sensor streaming to stabilize

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
        .quadwp_io_num = -1,  // Not used
        .quadhd_io_num = -1,  // Not used
        .max_transfer_sz = 4096  // Maximum transfer size in bytes
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,  // MHz
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

void captureImage(void) {

    debugSPI();

   // Clear FIFO pointers
    write_spi(0x04, 0x10);  // Reset FIFO write pointer
    write_spi(0x04, 0x20);  // Reset FIFO read pointer

    // Enable FIFO mode
    write_spi(0x03, 0x10);  // Enable FIFO mode (set Bit[4])

    // Start capture
    write_spi(0x04, 0x02);  // Start capture

    // Wait for capture completion
    uint8_t status = 0;
    for (int i = 0; i < 10; i++) {
        status = read_spi(0x41);
        if (status & 0x08) {  // FIFO write done flag
            ESP_LOGI(TAG, "Image capture completed.");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!(status & 0x08)) {
        ESP_LOGE(TAG, "Capture failed: FIFO not ready.");
        return;
    }

    // Get FIFO size
    uint32_t size = (read_spi(0x44) << 16) | (read_spi(0x43) << 8) | read_spi(0x42);
    ESP_LOGI(TAG, "Captured image size: %u bytes", (unsigned int) size);

    // Read data from FIFO
    uint8_t buffer[256];
    uint32_t bytes_read = 0;
    while (bytes_read < size) {
        uint32_t chunk = (size - bytes_read > sizeof(buffer)) ? sizeof(buffer) : (size - bytes_read);
        spi_transaction_t trans = {
            .length = chunk * 8,  // Bits
            .rx_buffer = buffer
        };
        esp_err_t ret = spi_device_transmit(spi, &trans);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Read %u bytes from FIFO.", (unsigned int) chunk);
            bytes_read += chunk;
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
