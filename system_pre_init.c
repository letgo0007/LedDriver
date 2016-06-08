/******************************************************************************
 * @file 	[system_pre_init.c]
 *
 * Boot with ISP(In-System Programming) function .
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 *****************************************************************************/
/***1 Includes ***************************************************************/

#include "msp430.h"

#include "std.h"

/***2.1 Internal Marcos ******************************************************/

//ISP mode I2C slave address
#define ISP_I2C_SLAVE_ADDRESS			(0x1C)

/***2.2 Internal Struct ******************************************************/

/***2.3 Internal Variables ***************************************************/

/***2.4 External Variables ***************************************************/

/***2.5 Internal Functions ***************************************************/
/**********************************************************
 * @Brief SetVcoreUp
 * 		Set Mcu Vcore Up for higher cpu speed.Refer to TI sample code.
 * 		Set Vcore level up 1 step up at a time another.
 * 		Use volatile inline to force compiler to build this function in
 * 		_system_pre_init()
 * @Param
 * 		level : available from 1~3 , refer to device specsheet for more info.
 * @Return
 * 		NONE
 **********************************************************/
volatile inline void SetVcoreUp(unsigned int level)
{
	// Open PMM registers for write
	PMMCTL0_H = PMMPW_H;
	// Set SVS/SVM high side new level
	SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
	// Set SVM low side to new level
	SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
	// Wait till SVM is settled
	while ((PMMIFG & SVSMLDLYIFG) == 0)
		;
	// Clear already set flags
	PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
	// Set VCore to new level
	PMMCTL0_L = PMMCOREV0 * level;
	// Wait till new level reached
	if ((PMMIFG & SVMLIFG))
	{
		while ((PMMIFG & SVMLVLRIFG) == 0)
		{
			;
		}
	}
	// Set SVS/SVM low side to new level
	SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
	// Lock PMM registers for write access
	PMMCTL0_H = 0x00;
}

//Write ISP_INIT_EXIT_FLAG to flash to exit isp mode on 1st run.
//Note that need open the project Properties - Debug - MSP43x Options - Erase Options = Enable Erase Flash & Info
#pragma LOCATION(ISP_INIT_EXIT_FLAG,ISP_EXIT_FLAG_ADDRESS)
volatile static const uint32 ISP_INIT_EXIT_FLAG = 0x20140217;
/**********************************************************
 * @Brief _system_pre_init
 * 		System low level initial before RAM intialization & main().
 * 		Use as isp mode boot loader.
 * 		DO NOT call any togher functions . If have to ,build the fucntion
 * 		with volatile inline . If this function is erased during ISP
 * 		process , it will cause system failure. Note that *,/ operation are
 * 		also external fuctions , do not use *,/ operation in
 * @Param
 * 		NONE
 * @Return
 * 		0	: omit RAM initialization.
 * 		1	: do RAM initialization.
 **********************************************************/
#pragma LOCATION(_system_pre_init,ISP_BOOT_ADDRESS)
int _system_pre_init(void)
{
	//Watch dog to 1sec .Clk source = ACLK , length = 32k.
	WDTCTL = WDTPW + WDTCNTCL + WDTHOLD + WDTSSEL_1 + WDTIS_4;

	//All gpio reset to Input with Pull Down.
	PAOUT = 0;
	PBOUT = 0;
	PCOUT = 0;
	PDOUT = 0;
	PADIR = 0;
	PBDIR = 0;
	PCDIR = 0;
	PDDIR = 0;
	PAREN = 1;
	PBREN = 1;
	PCREN = 1;
	PDREN = 1;

	//Disable ISR because Interrupt vetore @ 0xffd0~0xffff will also be erased in ISP mode.
	__disable_interrupt();

	//Check password on info flash to decide whether go to main().
	if (ISP_INIT_EXIT_FLAG == ISP_EXIT_PASSWORD32) //Password correct , go to main()
	{
		return 1;
	}
	else //Password wrong , run ISP function.
	{
		//Initialize buffers .
		//The flash segment size = 512 for MSP430 device ,so data buffer max size =512.
		//Flash Address is 32bit , address byte buffer size = 4 bytes .
		uint8 data_buff[512] =
		{ 0 };
		uint8 op_add_buff[4] =
		{ 0 };
		uint32 op_add = 0;
		uint8 *op_ptr = 0;
		uint16 txcount = 0;
		uint16 rxcount = 0;

		//Initialize CPU CLK to 25MHz
		SetVcoreUp(0x01);
		SetVcoreUp(0x02);
		SetVcoreUp(0x03);

		UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
		UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

		__bis_SR_register(SCG0);                  // Disable the FLL control loop
		UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
		UCSCTL1 = DCORSEL_7;                      // Select DCO range 50MHz operation
		UCSCTL2 = FLLD_1 + 762;                   // Set DCO Multiplier for 25MHz
												  // (N + 1) * FLLRef = Fdco
												  // (762 + 1) * 32768 = 25MHz
												  // Set FLL Div = fDCOCLK/2
		__bic_SR_register(SCG0);                  // Enable the FLL control loop
		__delay_cycles(782000);

		// Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
		do
		{
			UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
			// Clear XT2,XT1,DCO fault flags
			SFRIFG1 &= ~OFIFG;                      // Clear fault flags
		} while (SFRIFG1 & OFIFG);                   // Test oscillator fault flag

		//P4.6 = ERROR_OUT , P4.7 = LED_G , P3.0 = SDA ,P3.1 = SCL
		P4OUT |= BIT7 + BIT6;
		P4DIR |= BIT7 + BIT6;
		P3SEL |= BIT0 + BIT1;

		//Initialize I2C slave in ISP mode address ,without ISR function.
		UCB0CTL1 |= UCSWRST;
		UCB0CTL0 = UCMODE_3 + UCSYNC;
		UCB0I2COA = ISP_I2C_SLAVE_ADDRESS;
		UCB0CTL1 &= ~UCSWRST;

		while (1)
		{
			//Feed watch dog
			uint8 newWDTStatus = (WDTCTL & 0x00FF) | WDTCNTCL;
			WDTCTL = WDTPW + newWDTStatus;

			//YZF 2016/6/2 : Buffer UCB0IFG first to avoid UCB0IFG modified during processing .
			//				 This make sure I2C events are handled in order.
			//				 For some master device , RX/STOP , TX/START may come very close in time.
			uint16 UCB0IFG_BUFF = UCB0IFG;

			//I2C Slave RX Event (Master send bytes)
			if (UCB0IFG_BUFF & UCRXIFG)
			{
				//Buffer I2C slave data
				if (rxcount < 3)	// 0~2 bytes are operation address bytes.
				{
					op_add_buff[2 - rxcount] = UCB0RXBUF;

				}
				else	// 3+ bytes are data bytes.
				{
					data_buff[rxcount - 3] = UCB0RXBUF;
				}

				rxcount++;
				UCB0IFG &= ~UCRXIFG;
			}

			//I2C Slave START Event (Master call slave address)
			if (UCB0IFG_BUFF & UCSTTIFG)
			{
				//Calculate operation address & pointer , for "Restart" conditon.
				op_add = (*((volatile uint32 *) &op_add_buff[0]));
				op_ptr = (uint8 *) (op_add & 0x0FFFFF);

				//Reset count
				txcount = 0;
				rxcount = 0;
				UCB0IFG &= ~UCSTTIFG;
			}

			//I2C Slave STOP Event (Master send STOP)
			if (UCB0IFG_BUFF & UCSTPIFG)
			{
				//Calculate operation address & pointer
				op_add = (*((volatile uint32 *) &op_add_buff[0]));
				op_ptr = (uint8 *) (op_add & 0x0FFFFF);

				/* Flash operation according to opp_add
				 * Address				Operation
				 * -----------------	----------
				 * 0x104400~0x124400 	ERASE
				 * 0x004400~0x024400	WRITE
				 * 0xFFFFFF				EXIT ISP
				 *
				 */
				if ((op_add >= 0x104400) && (op_add <= 0x124400))	//ERASE
				{
					FCTL3 = FWKEY;
					FCTL1 = FWKEY + ERASE;
					*op_ptr = 0xFF;
					FCTL1 = FWKEY;
					FCTL3 = FWKEY + LOCK;
				}
				if ((rxcount > 2) && (op_add >= 0x004400) && (op_add <= 0x024400))	//WRITE
				{
					uint16 i;
					uint8 * flash_ptr;
					flash_ptr = (uint8 *) op_add;

					FCTL3 = FWKEY;
					FCTL1 = FWKEY + WRT;
					for (i = 0; i < rxcount - 3; i++)
					{
						*(flash_ptr + i) = data_buff[i];
					}
					FCTL1 = FWKEY;
					FCTL3 = FWKEY + LOCK;
				}
				if (op_add == 0xFFFFFF)	//EXIT ISP
				{
					//Write ISP EXIT Password.
					unsigned long * flash_ptr;
					flash_ptr = (unsigned long *) ISP_EXIT_FLAG_ADDRESS;
					//Erase segment first
					FCTL3 = FWKEY;
					FCTL1 = FWKEY + ERASE;
					*flash_ptr = 0;
					//Long-word(32bit) write
					FCTL1 = FWKEY + BLKWRT;
					*flash_ptr = ISP_EXIT_PASSWORD32;
					//Lock Flash
					FCTL1 = FWKEY;
					FCTL3 = FWKEY + LOCK;

					//Reboot
					PMMCTL0 |= PMMSWBOR;
				}
				UCB0IFG &= ~UCSTPIFG;
			}

			//I2C Slave TX Event (Master Read)
			if (UCB0IFG_BUFF & UCTXIFG)
			{
				UCB0TXBUF = op_ptr[txcount];
				txcount++;
			}
		}	//End of while (1)
	}
}

/***2.6 External Functions ***************************************************/
