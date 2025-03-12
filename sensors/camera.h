#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

// SPI configuration
#define CS_PIN 5         // Chip Select GPIO
#define SCLK_PIN 18      // SPI Clock GPIO
#define MOSI_PIN 23      // Master Out Slave In GPIO
#define MISO_PIN 19      // Master In Slave Out GPIO

// OV5642 Registers
#define ARDUCHIP_VERSION        0x40  //ArduChip Version (Mini_5mp_plus: 0x73)
#define ARDUCHIP_MODE      		0x02  //BUS Mode register
#define ARDUCHIP_FRAMES			0x01  //FRAME control register, Bit[2:0] = Number of frames to be captured + 1

#define ARDUCHIP_FIFO           0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02

#define BURST_FIFO_READ			0x3C  //Burst FIFO read operation
#define SINGLE_FIFO_READ		0x3D  //Single FIFO read operation
#define CAM_WR_FIFO_DONE        0x41  //Bit[3]: camera write FIFO done flag (0x08)

#define ARDUCHIP_GPIO_WRITE		0x06  //GPIO Write Register
#define ARDUCHIP_GPIO_READ		0x45  //GPIO Write Register
#define GPIO_DIRECTION			0x05  //GPIO Direction Register
#define GPIO_RESET_MASK			0x01  //0 = Sensor reset,				1 = Sensor normal operation
#define GPIO_PWDN_MASK			0x02  //0 = Sensor normal operation, 	1 = Sensor standby
#define GPIO_PWREN_MASK			0x04  //0 = Sensor LDO disable, 		1 = sensor LDO enable
#define FIFO_SIZE1				0x42  //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2				0x43  //Camera write FIFO size[15:8]
#define FIFO_SIZE3				0x44  //Camera write FIFO size[18:16]





void setupCamera(void);
void captureImage(void);
void setupSPI(void);
void reset_camera_via_spi(void);
#endif // CAMERA_SETUP_H
