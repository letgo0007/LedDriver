#include "driver_mcu.h"
#include "std.h"
#include "driverlib.h"

void Mcu_Boot(void)
{
	if (HREG32(0x1900) == 0x20140217)
	{
		/*Application*/;
	}
	else
	{
		__disable_interrupt();
		//P4.6 = ERROR_OUT , P4.7 = LED_G , P3.0 = SDA ,P3.1 = SCL
		P4OUT |= BIT7 + BIT6;
		P4DIR |= BIT7 + BIT6;
		P3SEL |= BIT0 + BIT1;

		//Initialize I2C slave in ISP mode address
		UCB0CTL1 |= UCSWRST;
		UCB0CTL0 = UCMODE_3 + UCSYNC;
		UCB0I2COA = 0x28;
		UCB0CTL1 &= ~UCSWRST;

		static uint8 data_buff[512];
		static uint8 op_buff[4];
		static uint16 txcount;
		static uint16 rxcount;
		static uint32 op_add;
		static uint8 *op_ptr;

		while (1)
		{
			//RX
			if (UCB0IFG & UCRXIFG)
			{
				//Buffer I2C slave data
				if (rxcount < 3)
				{
					op_buff[2 - rxcount] = UCB0RXBUF;
				}
				else
				{
					data_buff[rxcount - 3] = UCB0RXBUF;
				}

				rxcount++;
				UCB0IFG &= ~UCRXIFG;
			}
			//TX
			if (UCB0IFG & UCTXIFG)
			{
				UCB0TXBUF = op_ptr[txcount];
				txcount++;
				UCB0IFG &= ~UCTXIFG;
			}
			//START
			if (UCB0IFG & UCSTTIFG)
			{
				//Calculate operation address & pointer
				op_add = (*((volatile uint32 *) &op_buff[0]));
				op_ptr = (uint8 *) (op_add & 0x0FFFFF);

				//Reset count
				txcount = 0;
				rxcount = 0;
				UCB0IFG &= ~UCSTTIFG;
			}
			//STOP
			if (UCB0IFG & UCSTPIFG)
			{
				//Calculate operation address & pointer
				op_add = (*((volatile uint32 *) &op_buff[0]));
				op_ptr = (uint8 *) (op_add & 0x0FFFFF);

				//Flash operation according to opp_add
				//Segment Erase
				if ((op_add >= 0x104400) && (op_add <= 0x124400))
				{
					FCTL3 = FWKEY;                            // Clear Lock bit
					FCTL1 = FWKEY + ERASE;                      // Set Erase bit
					*op_ptr = 0xFF;
					FCTL1 = FWKEY;                            // Clear WRT bit
					FCTL3 = FWKEY + LOCK;                       // Set LOCK bit
				}
				//Segment Write
				if ((rxcount > 2) && (op_add >= 0x004400) && (op_add <= 0x024400))
				{
					uint16 i;
					uint8 * Flash_ptr;                     // Initialize Flash pointer
					Flash_ptr = (uint8 *) op_add;

					FCTL3 = FWKEY;                            // Clear Lock bit
					FCTL1 = FWKEY + ERASE;                      // Set Erase bit
					*Flash_ptr = 0;                           // Dummy write to erase Flash seg
					FCTL1 = FWKEY + WRT;                        // Set WRT bit for write operation
					for (i = 0; i < rxcount - 3; i++)
					{
						*Flash_ptr++ = data_buff[i];        // Write value to flash
					}
					FCTL1 = FWKEY;                            // Clear WRT bit
					FCTL3 = FWKEY + LOCK;                       // Set LOCK bit
				}
				//Reboot
				if (op_add == 0xFFFFFF)
				{                       //Reboot CMD
					PMMCTL0 |= PMMSWBOR;
				}

				UCB0IFG &= ~UCSTPIFG;

			}

		}

	}

}

