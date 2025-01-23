#ifndef I2CSETUP_H
#define I2CSETUP_H

#include <stdint.h>  // For standard integer types

void setupI2C(void);
void sendI2CCommand(uint8_t reg, uint8_t value);
void i2cScanner(void);

#endif // I2CSETUP_H
