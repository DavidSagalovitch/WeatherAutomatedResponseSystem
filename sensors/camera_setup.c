#include "camera.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "ov5642_regs.h"



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
    /*
    write_i2c(0x3818, 0x81); // Enable JPEG output format

    for (int i = 0; ov5642_320x240[i].reg != 0xFFFF; i++) {
            write_i2c(ov5642_320x240[i].reg, ov5642_320x240[i].val);
            ESP_LOGI(TAG, "Wrote 0x%02X to register 0x%04X", ov5642_320x240[i].val, ov5642_320x240[i].reg);
            vTaskDelay(pdMS_TO_TICKS(10)); // Optional delay for stability
        }
    ESP_LOGI(TAG, "Camera initialization complete.");

    // Start streaming to activate the sensor
    write_i2c(0x3008, 0x02);  // Start sensor streaming
    vTaskDelay(pdMS_TO_TICKS(200));  // Allow sensor streaming to stabilize
    */
   
   // Reset all registers to default
   write_i2c(0x3008, 0x80);  // Soft reset
   vTaskDelay(pdMS_TO_TICKS(100));
   
   // Set to JPEG output mode
   write_i2c(0x3818, 0xA8);  // Enable JPEG output format and YUV422
   write_i2c(0x3035, 0x11);  // PLL settings
   write_i2c(0x3036, 0x69);  // PLL multiplier
   write_i2c(0x3C07, 0x07);  // Enable color bar test pattern
   
   // Configure more registers if needed based on your datasheet
   
   // Apply the resolution settings
   for (int i = 0; ov5642_320x240[i].reg != 0xFFFF; i++) {
       write_i2c(ov5642_320x240[i].reg, ov5642_320x240[i].val);
       vTaskDelay(pdMS_TO_TICKS(10));
   }
   
   // Start streaming
   write_i2c(0x3008, 0x02);
   vTaskDelay(pdMS_TO_TICKS(200));
   // Confirm sensor is detected
   uint8_t vid = read_i2c(0x300A);  // Read Vendor ID
   uint8_t pid = read_i2c(0x300B);  // Read Product ID
   if (vid == 0x56 && pid == 0x42) {
       ESP_LOGI(TAG, "OV5642 detected. VID: 0x%02X, PID: 0x%02X", vid, pid);
   } else {
       ESP_LOGE(TAG, "OV5642 not detected. VID: 0x%02X, PID: 0x%02X", vid, pid);
   }
   ESP_LOGI(TAG, "Camera I2C setup completed");
}

// SPI device handle
spi_device_handle_t spi;

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
        uint8_t readback = read_spi(reg);
        if (readback == value){
            ESP_LOGI(TAG, "Successfully read back 0x%02X from SPI register 0x%02X", value, reg);
        } else {
            ESP_LOGE(TAG, "Failed to read back 0x%02X from SPI register 0x%02X", value, reg);
        }
    }
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
        .mode = 0,               // SPI mode 0
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
    write_spi(ARDUCHIP_FIFO, 0x20);
    vTaskDelay(pdMS_TO_TICKS(10));
    value = read_spi(ARDUCHIP_FIFO);
    ESP_LOGI(TAG, "FIFO Control Register (0x04) after clearing: 0x%02X", value);

    write_spi(ARDUCHIP_FIFO, 0x10);
    vTaskDelay(pdMS_TO_TICKS(10));
    value = read_spi(ARDUCHIP_FIFO);
    ESP_LOGI(TAG, "FIFO Control Register (0x04) after write pointer reset: 0x%02X", value);

    write_spi(ARDUCHIP_FIFO, 0x08);
    vTaskDelay(pdMS_TO_TICKS(10));
    value = read_spi(ARDUCHIP_FIFO);
    ESP_LOGI(TAG, "FIFO Control Register (0x04) after read pointer reset: 0x%02X", value);

    write_spi(ARDUCHIP_FIFO, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    value = read_spi(ARDUCHIP_FIFO);
    ESP_LOGI(TAG, "FIFO Control Register (0x04) after enabling: 0x%02X", value);

    // Test reading FIFO Status Register
    value = read_spi(CAM_WR_FIFO_DONE);
    printf("%02X", (unsigned)value);
    if (value == 0x00){
        ESP_LOGI(TAG, "FIFO Status Register (0x41): 0x%02X", value);
        ESP_LOGI(TAG, "FIFO DONE FLAG RESET");
    } else {
        ESP_LOGE(TAG, "FIFO Status Register (0x41): 0x%02X", value);
        ESP_LOGE(TAG, "FIFO DONE FLAG NOT RESET");
    }
}

void reset_camera_via_spi(void) {
    ESP_LOGI(TAG, "Starting SPI reset sequence for the camera...");
    write_spi(ARDUCHIP_FIFO, 0x20);  // Reset FIFO write pointer
    vTaskDelay(pdMS_TO_TICKS(50));
    write_spi(ARDUCHIP_FIFO, 0x10);  // Reset FIFO write pointer
    vTaskDelay(pdMS_TO_TICKS(50));
    write_spi(ARDUCHIP_FIFO, 0x08);  // Reset FIFO read pointer
    vTaskDelay(pdMS_TO_TICKS(50));
    write_spi(ARDUCHIP_FIFO, 0x01);  // Clear FIFO write done flag
    vTaskDelay(pdMS_TO_TICKS(50));
    write_spi(0x05, 0x07);  // Set GPIOs as output
    vTaskDelay(pdMS_TO_TICKS(50));
    write_spi(0x03, 0x00);  // Disable FIFO mode, default Hsync/Vsync
    vTaskDelay(pdMS_TO_TICKS(50));
    write_spi(0x01, 0x00);  // Stop capture
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "Camera reset sequence via SPI completed.");
}

void captureImage(void) {
    // Reset camera hardware
    write_spi(GPIO_DIRECTION, 0x00);
    write_spi(ARDUCHIP_GPIO, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
    write_spi(ARDUCHIP_GPIO, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
    reset_camera_via_spi();
    //debugSPI();
    
    // Start single frame capture
    write_spi(ARDUCHIP_FRAMES, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
    read_spi(ARDUCHIP_FRAMES);
    write_spi(ARDUCHIP_FIFO, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
    write_spi(ARDUCHIP_FIFO, 0x02);
    
    // Wait for capture complete
    ESP_LOGI(TAG, "Waiting for image capture...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    uint8_t status = 0;
    uint32_t timeout = 20;  // Increased timeout
    
    for (uint32_t i = 0; i < timeout; i++) {
        status = read_spi(CAM_WR_FIFO_DONE);
        ESP_LOGI(TAG, "FIFO Status (0x41): 0x%02X", (unsigned int) status);
        
        if (status & 0x08) {
            ESP_LOGI(TAG, "Image capture completed after %u attempts", (unsigned int) i + 1);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Improved FIFO read method
    uint8_t s1 = read_spi(0x44);
    uint8_t s2 = read_spi(0x43);
    uint8_t s3 = read_spi(0x42);
    uint32_t size = (s1 << 16) | (s2 << 8) | s3;
    ESP_LOGI(TAG, "Size bytes: %02X %02X %02X -> %u bytes", (unsigned int)s1, (unsigned int)s2, (unsigned int)s3, (unsigned int)size);

    if (size > 0 && size < 100000) {  // Sanity check on size
        uint8_t *buffer = malloc(size);
        if (buffer) {
            // Select FIFO read register
            write_spi(BURST_FIFO_READ, 0x00);
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Read the data byte by byte
            for (uint32_t i = 0; i < size; i++) {
                // Setup a read transaction
                spi_transaction_t trans = {
                    .length = 8,  // 8 bits (1 byte)
                    .flags = SPI_TRANS_USE_RXDATA,
                    .rx_data = {0}
                };
                
                if (spi_device_transmit(spi, &trans) == ESP_OK) {
                    buffer[i] = trans.rx_data[0];
                    if (i < 32 || i > size - 32) {  // Print first and last 32 bytes
                        printf("%02X ", buffer[i]);
                        if ((i + 1) % 16 == 0)
                            printf("\n");
                    }
                } else {
                    ESP_LOGE(TAG, "SPI read error at byte %u", (unsigned int)i);
                    break;
                }
            }
            printf("\n");
            free(buffer);
        } else {
            ESP_LOGE(TAG, "Failed to allocate memory for image data");
        }
    } else {
        ESP_LOGE(TAG, "Invalid image size: %u bytes", (unsigned int)size);
    }
    
    reset_camera_via_spi();
        vTaskDelay(pdMS_TO_TICKS(1000)); 
}


void setupCamera(void) {   
    
    setupSPI();
    vTaskDelay(pdMS_TO_TICKS(100)); 
    reset_camera_via_spi();
    vTaskDelay(pdMS_TO_TICKS(100)); 
    setup_camera_I2C();
    vTaskDelay(pdMS_TO_TICKS(1000)); 
}
