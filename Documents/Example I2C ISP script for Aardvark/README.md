# I2C ISP script examples
##1 I2C protocol introduction.
I2C slave of this project has 2 modes: [NORMAL] & [ISP].
[NORMAL] slave address is 0x28 , address lenth = 1 (from 0x00~0xFF) , with access to I2C Slave Buffer in RAM.
[ISP] 	slave address is 0x38 , address length = 3 (from 0x00~0xFFFFFF), with access to all address in MCU, including RAM/ROM/SFR.

Example 1:
[ISP] Write 3 bytes (0xAA 0xBB 0xCC) to address (0x112233)
START - DA	- 	{ADD - 	ADD - 	ADD} - 	{DATA - 	DATA - 	DATA} - STOP
		0x38		0x11		0x22		0x33		0xAA		0xBB		0xCC
Example 2:	
[ISP] Write 3 bytes from address (0x112233)
START - DA	- 	{ADD - 	ADD - 	ADD} - 	START - {DATA - 	DATA - 	DATA} - STOP
		0x38		0x11		0x22		0x33				xx		xx		xx
Example 3:
[NORMAL] Write 3 bytes (0xAA 0xBB 0xCC) to address (0x11)
START - DA	- 	{ADD} - 	{DATA - 	DATA - 	DATA} - STOP
		0x38		0x11		0xAA		0xBB		0xCC
Example 4:	
[NORMAL] Write 3 bytes from address (0x112233) in [ISP]
START - DA	- 	{ADD} - 	START - {DATA - 	DATA - 	DATA} - STOP
		0x38		0x11				xx		xx		xx

##2 [NORMAL] <-> [ISP] entrance & exit.
###2.1 [NORMAL] -> [ISP]
1 Write [NORMAL] -> [ISP] password "0x0217" to address [0xFE] [0xFF]
If pass word correct , I2C slave address will switch to 0x38 , 0x28 will has no ACK when called.
2 Erase [ISP] -> [NORMAL] password @ [0x00F000] .
Once Erased , System will always boot to ISP mode until correct pass word is written.
3 Reboot , Write the address [0xFFFFFF] in [ISP].

###2.1 [ISP] -> [NORMAL]
1 Write [ISP] -> [NORMAL] password "0x20140217" to address [0x00F000] .
2 Reboot , Write the address [0xFFFFFF] in [ISP].

