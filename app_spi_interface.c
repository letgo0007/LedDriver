/*****************************************************************************
 * @file 	[driver_uartdebug.h]
 *
 * Debug uart port function headers.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 *****************************************************************************/

#include "app_spi_interface.h"

/******************************************************************************
 * SpiSlave_handle
 * @Brief
 * 		Convert SPI data into duty matrix.
 * @Variables
 * 		*spirx				: SPI hardware module recieved data
 * 		*outputduty			: Duty Matrix Output
 * 		SpiRxFormatModel	: SPI format model selection
 * @Return
 * 		FLAG_SUCCESS		: SPI data format OK, convert finish.
 * 		FLAG_FAIL			: SPI data format NG, no output data.
 *
 *****************************************************************************/
uint8 SpiSlave_handle(uint8 *spirx, uint16 *outputduty, enum SpiRxFormatModel model)
{
	switch (model)
	{
	case HISIV600_8BIT:
	{
		/* HisiV600 SPI format:
		 * 	Header			Lengh		Data(8bit)				Checksum=(Header + length + data)&0xFF
		 * 	0x01 0x40		0x03		0x01 0x02 0x03  		0x4A
		 * 	[0]	 [1]		[2]			[3]  [..]  [2+Length]	[3+length]
		 */
		//Step 1 : Check Header
		if ((spirx[0] == 0x01) && (spirx[1] == 0x40) && (spirx[2] >= 0x01))
		{
			uint8 length = spirx[2];
			uint8 checksum = spirx[3 + length];
			uint8 i = 0;
			uint8 sum = 0;

			//Step 2 : Check Sum
			for (i = 0; i < length + 2; i++)
			{
				sum = sum + spirx[i];
			}

			//Step 3: Transfer data
			//if(sum == checksum)
			if (checksum == checksum)
			{
				for (i = 0; i < length; i++)
				{
					outputduty[i] = spirx[3 + i] * 0x0010;
				}
				return FLAG_SUCCESS;
			}
			//Error handle
			else
			{
				return FLAG_FAIL;
			}

		}
		else
		{
			return FLAG_FAIL;
		}
	} //End of Case HISIV600_8BIT

	case MFC11_12BIT_80CH:
	{
		/* MFC11 spi format: Data = 12bit
		 * 					0	 1	  2    3    4    5
		 * SPI :			0x12 0x34 0x56 0x78 0x9A 0xBC
		 * Duty:			0x123    0x456 0x789    0xABC
		 * Channel:         0		 1     2        3
		 */
		uint16 i = 0;
		uint32 temp = 0;
		for (i = 0; i < 120; i = i + 3)	//120ch 8bit = 80ch 12bit
		{
			temp = 0x010000 * spirx[i] + 0x000100 * spirx[i + 1] + spirx[i + 2];
			outputduty[i / 3] = temp >> 12;
			outputduty[i / 3 + 1] = temp & 0xFFF;
		}
		return FLAG_SUCCESS;
	}	//End of Case MFC11_12BIT_80CH

	case CITRUS_12BIT_78CH:
	{
		/* Citrus spi format:
		 *
		 * 						[Head]	[D0]	[D1]	[D2]	[D3]	[D4]	[D5]
		 * SPI :		[Row0]	960 	123123 	456456 	789789 	ABCABC 	DEFDEF 	123123	= 19.5 byte
		 * 				[Row1]	961 	123123 	456456 	789789 	ABCABC 	DEFDEF 	123123	= 19.5 byte
		 * 				...
		 * 				[Row12]	96C 	123123 	456456 	789789 	ABCABC 	DEFDEF 	123123	= 19.5 x 13 = 253.5 byte
		 *
		 * SPI count	[Row0]	0 1		 2 3 4	 5 6 7   8 9 A   B C D	 E F G	 H I J
		 * 				[Row1]   K		L M N	O P Q 	R S T 	U V W	X Y Z	0 1 2
		 * 				[Row2]  3 4
		 *
		 * Duty: 						0x123 	0x456 	0x789 	0xABC 	0xDEF 	0x123
		 *
		 * Transfer:	Duty[row = 0 , col = 0] = SPI[2] + 0x0100 * (SPI[3]>>4)
		 * 				Duty[row = 0 , col = 1] = SPI[5] + 0x0100 * (SPI[6]>>4)
		 * 				...
		 * 				Duty[row = 0 , col = 5] = SPI[col*3+2] + 0x0100 * (SPI[col*3+3]>>4)
		 *
		 * 				Duty[row = 1 , col = 0] = 0x0010 * SPI[21] + (SPI[22]>>4)
		 * 				Duty[row = 1 , col = 1] = 0x0010 * SPI[24] + (SPI[25]>>4)
		 * 				...
		 * 				Duty[row = 1 , col = 5] = 0x0010 * SPI[col*3 + 21] + (SPI[col*3 + 22]>>4)
		 *
		 * 				Duty[row = 2 , col = 0] = SPI[2 + 40] + 0x0100 * (SPI[3 + 40]>>4)
		 *
		 */

		//check header
		if (spirx[0] != 0x96)
		{
			return FLAG_FAIL;
		}

		uint16 row, col;
		//col = 6 , row = 13 matrix
		for (row = 0; row < 13; row++)
		{
			for (col = 0; col < 6; col++)
			{
				//row 1,3,5,7,9,11
				if (row & 0x1)
				{
					outputduty[row * 6 + col] = (0x0010 * spirx[row / 2 * 39 + col * 3 + 21]
							+ (spirx[row / 2 * 39 + col * 3 + 22] >> 4));
				}
				//row 0,2,4,6,8,10,12
				else
				{
					outputduty[row * 6 + col] = (spirx[row / 2 * 39 + col * 3 + 2]
							+ 0x0100 * (spirx[row / 2 * 39 + col * 3 + 3] >> 4));

				}
			}
		}
	}

		return FLAG_SUCCESS;

	default:
		//undefined SPI format , return fail
		return FLAG_FAIL;
	}
}

