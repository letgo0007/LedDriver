#ifndef APP_I2C_INTERFACE_H_
#define APP_I2C_INTERFACE_H_
#include "driverlib.h"

//I2C Function 1 [SPI access function]

/* Value @ I2C_SPIACCESS_WRMODE
 * [00] No operation (default)
 * [80] Single Read
 * [81] Single Write
 * [82] Multiple Read
 * [84] Multiple Write
 */
#define I2C_SPIACCESS_WRMODE		(0x00)
#define I2C_SPIACCESS_CHIPSEL		(0x01)
#define I2C_SPIACCESS_REGADD		(0x02)
#define I2C_SPIACCESS_TXDATA		(0x03)
#define I2C_SPIACCESS_RXDATA		(0x04)

//I2C Function 5 [Manual pattern]
/* Value @ I2C_MANUAL_WRMODE
 * [00] Read param from System_ManualDutyBuff (default)
 * [80] Write param to System_ManualDutyBuff
 */
#define I2C_MANUAL_WRMODE			(0x10)
#define I2C_MANUAL_DUTY_H			(0x11)
#define I2C_MANUAL_DUTY_L			(0x12)
#define I2C_MANUAL_START_CH			(0x13)
#define I2C_MANUAL_END_CH			(0x14)


//Function calls
uint8 I2cSlave_handleSpecialFunction(uint8 *i2cmap);
uint8 I2cSlave_initMap(uint8 *i2cmap);

#endif /* APP_I2C_INTERFACE_H_ */
