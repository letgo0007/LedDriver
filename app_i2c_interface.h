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
#define I2C_SPIACCESS_WRMODE		(0x80)
#define I2C_SPIACCESS_CHIPSEL		(0x81)
#define I2C_SPIACCESS_REGADD		(0x82)
#define I2C_SPIACCESS_LENGTH		(0x83)

//I2C Function 2 [Scheduler control]
/* Value @ I2C_SCH_WRMODE
 * [00] Read param from System_Schedule (default)
 * [80] Write param to System_Schedule
 */
#define I2C_SCH_WRMODE				(0x90)
#define I2C_SCH_RESET				(0x91)
#define I2C_SCH_LDENABLE			(0x92)
#define I2C_SCH_PEROID_CPUTICK_H	(0x93)
#define I2C_SCH_PEROID_CPUTICK_L	(0x94)
#define I2C_SCH_PEROID_GPIOCHECK_H	(0x95)
#define I2C_SCH_PEROID_GPIOCHECK_L	(0x96)
#define I2C_SCH_PEROID_MANUALMODE_H	(0x97)
#define I2C_SCH_PEROID_MANUALMODE_L	(0x98)
#define I2C_SCH_CPULOAD				(0x99)

//I2C Function 3 [Backlight control]
/* Value @ I2C_BL_WRMODE
 * [00] Read param from System_Iw7027Param (default)
 * [80] Write param to System_Iw7027Param
 */
#define I2C_BL_WRMODE				(0xA0)
#define I2C_BL_IWFREQ				(0xA1)
#define I2C_BL_IWCURRENT			(0xA2)
#define I2C_BL_IWDELAYSEL			(0xA3)
#define I2C_BL_VSYNCFREQ			(0xA4)
#define I2C_BL_VSYNCDELAY			(0xA5)

//I2C Function 4 [Error control]
/* Value @ I2C_BL_WRMODE
 * [00] Read param from System_ErrorParam (default)
 * [80] Write param to System_ErrorParam
 */
#define I2C_ERROR_WRMODE			(0xB0)
#define I2C_ERROR_ERRORTYPE			(0xB1)
#define I2C_ERROR_ERRORCOUNT		(0xB2)
#define I2C_ERROR_DC60VMAX			(0xB3)
#define I2C_ERROR_DC60VMIN			(0xB4)
#define I2C_ERROR_DC13VMAX			(0xB5)
#define I2C_ERROR_DC13VMIN			(0xB6)
#define I2C_ERROR_SPIRXFREQMIN		(0xB7)
#define I2C_ERROR_SPIDATACHECKEN	(0xB8)
#define I2C_ERROR_IW7027FAULTIGNORE	(0xB9)
#define I2C_ERROR_IW7027FAULTTYPE	(0xBA)
#define I2C_ERROR_ERRORSAVEEN		(0xBB)

//I2C Function 5 [Manual pattern]
/* Value @ I2C_MANUAL_WRMODE
 * [00] Read param from System_ManualDutyBuff (default)
 * [80] Write param to System_ManualDutyBuff
 */
#define I2C_MANUAL_WRMODE			(0xC0)
#define I2C_MANUAL_DUTY_H			(0xC1)
#define I2C_MANUAL_DUTY_L			(0xC2)
#define I2C_MANUAL_CHSTART			(0xC3)
#define I2C_MANUAL_CHAMOUNT			(0xC4)

//I2C Function 6 [DPL control - 1]
/* Value @ I2C_MANUAL_WRMODE
 * [00] Read param from System_DplParam (default)
 * [80] Write param to System_DplParam
 * [81] Load Default Param Table 1 to System_DplParam;
 * [82] Load Default Param Table 2 to System_DplParam;
 * ...
 * Const table amount refer to model introduction
 */
#define I2C_DPL_WRMODE				(0xD0)
#define I2C_DPL_ENABLE				(0xD1)
#define I2C_DPL_CH_AMOUNT			(0xD2)
#define I2C_DPL_GAMMA_SEL			(0xD3)
#define I2C_DPL_SAMPLE_FRAME		(0xD4)
#define I2C_DPL_UPDATE_FRAME_H		(0xD5)
#define I2C_DPL_UPDATE_FRAME_L		(0xD6)
#define I2C_DPL_LIMIT_UPSTEP_H		(0xD7)
#define I2C_DPL_LIMIT_UPSTEP_L		(0xD8)
#define I2C_DPL_LIMIT_DOWNSTEP_H	(0xD9)
#define I2C_DPL_LIMIT_DOWNSTEP_L	(0xDA)
#define I2C_DPL_GDMAX_H				(0xDB)
#define I2C_DPL_GDMAX_L				(0xDC)
#define I2C_DPL_LDMAX_H				(0xDD)
#define I2C_DPL_LDMAX_L				(0xDE)
#define I2C_DPL_HIGHTEMP_H			(0xDF)
#define I2C_DPL_HIGHTEMP_L			(0xE0)
#define I2C_DPL_LOWTEMP_H			(0xE1)
#define I2C_DPL_LOWTEMP_L			(0xE2)


//I2C Function 7 [Factory test] T.B.D
#define I2C_ISP_ENTRANCE			(0xF0)


//I2C Function 8 [Version Info]
#define I2C_SOFTWARE_YEAR			(0xFC)
#define I2C_SOFTWARE_MONTH			(0xFD)
#define I2C_SOFTWARE_DAY			(0xFE)
#define I2C_SOFTWARE_VER			(0xFF)

//Function calls
uint8_t I2cSlave_handleMap(uint8_t *i2cmap);
uint8_t I2cSlave_initMap(uint8_t *i2cmap);

#endif /* APP_I2C_INTERFACE_H_ */
