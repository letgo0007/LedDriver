/******************************************************************************
 * @file 	[app_i2c_interface.h]
 *
 * I2C slave interface Special Function Hander.
 * The I2C slave have 2 ways to do system control.
 * 1 : DMA Mode , i2c slave data direct access to RAM , modify params .
 * 2 : Special Function Mode , for some function, not all params is memory
 * acessable or memory too large, function is called in certain order.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 *****************************************************************************/

#ifndef DRV_I2C_SLAVE_H_
#define DRV_I2C_SLAVE_H_

/***1 Includes ****************************************************************/

#include "std.h"

/***2.1 External Macros ******************************************************/

// Address offset define for Special Function 1 [SPI direct access function]
#define I2C_SPIACCESS_WRMODE		(0x00)
#define I2C_SPIACCESS_CHIPSEL		(0x01)
#define I2C_SPIACCESS_REGADD		(0x02)
#define I2C_SPIACCESS_TXDATA		(0x03)
#define I2C_SPIACCESS_RXDATA		(0x04)

// Address offset define for Special Function 2 [Manual pattern]
#define I2C_MANUAL_WRMODE			(0x10)
#define I2C_MANUAL_DUTY_L			(0x11)
#define I2C_MANUAL_DUTY_H			(0x12)
#define I2C_MANUAL_START_CH			(0x13)
#define I2C_MANUAL_END_CH			(0x14)

// Address offset define for Special Function 3 [Direct Meemory Access]
#define I2C_DMA_WRMODE				(0x20)
#define I2C_DMA_ADD_L				(0x21)
#define I2C_DMA_ADD_H				(0x22)
#define I2C_DMA_TXDATA				(0x23)
#define I2C_DMA_RXDATA				(0x24)

/***2.2 External Structures **************************************************/

/***2.3 External Variables ***************************************************/

/***2.4 External Functions ***************************************************/

uint8 I2cSlave_handleSpecialFunction(uint8 *i2cmap);

#endif /* DRV_I2C_SLAVE_H_ */
