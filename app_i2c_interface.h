#ifndef APP_I2C_INTERFACE_H_
#define APP_I2C_INTERFACE_H_
#include "driverlib.h"

uint8_t I2cSlave_handleMap(uint8_t *i2cmap);
uint8_t I2cSlave_initMap(uint8_t *i2cmap);

#endif /* APP_I2C_INTERFACE_H_ */
