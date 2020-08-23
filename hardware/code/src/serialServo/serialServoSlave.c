
#include "serialServo/serialServoSlave.h"
#include "macros.h"

static uint16_t *cmdpos;
static uint16_t *cachepos;
static uint8_t *listen;

static uint8_t *servoConfig;

static uint16_t numServos;

// Comms
typedef enum servoFlags_e
{
	ENGCACHE = 0,
	SERVOON,
	SERVOOFF,
	CMD2CUR
} servoFlags_t;



#define Version 5

#define ReplyPos 0
#define ReplyCur 1
#define ReplyVer 2


void serialServoSlave_setup(uint16_t *cmdposAddr, uint16_t *cacheposAddr, uint8_t *listenAddr, uint8_t numServos_, const uint8_t *servoConfigAddr){
    cmdpos = cmdposAddr;
    cachepos = cacheposAddr;
    listen = listenAddr;
    numServos = numServos_;
    servoConfig = servoConfigAddr;
}


void debugToggle(void)
{
	//pass
}


unsigned int serialServoSlave_dobyte(uint8_t data)
{

	static unsigned char state = 0;
	static unsigned int command;
	static unsigned int argument;

	if (state == 0)
	{
		if (IsLow(7, data))
		{
			state = 1;
			command = (data >> 3);
			argument = (argument & 0xFFF8) | (data & 0x07); // glue in its 0 through 2
		}
	}
	else
	{
		state = 0;
		if (IsHigh(7, data))
		{
			argument = (argument & 0x0007) | ((data & 0x7F) << 3); //glue in bits 3 through 9
			return serialServoSlave_servoCmd(command, argument);
		}
	}

	return 0;
}

// 0  listen (servo number) 256 = all                      {always obey command} // sticks through listen once
// 1  ignore (servo number) 256 = all                      {always obey command} // overrides listen once
// 2  One Time listen (servo number)                       {always obey command}
// 3  set flags (flags) (+toggle debug)                    { bitwise obey }
//    0 enguage cached position                              {always obey command}
//    1 turn servo on                                        {obey if listening}
//    2 turn servo off                                       {obey if listening}
//    3 set cmdpos to curpos                                 {obey if listening}
// 4  set servo position (position)                        {obey if listening}
// 5  set cached position (position)                       {obey if listening}
// 6 get servo current  (servo number)                    {servo number}
// 7 get servo position (servo number)                    {servo number}
// 8 send device model  (servo number)                    {servo number}

unsigned int serialServoSlave_servoCmd(unsigned int command, unsigned int argument)
{

	unsigned int reply;
	static unsigned int chainAddress = 1023;

	reply = 0;

	switch (command)
	{

	case 0: // listen(id)
		chainAddress = 1023;
		if (argument == 256)
		{
			// Listen all
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				listen[servoIdx] |= 2;
			}
		}
		else
		{
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				if (argument == servoConfig[servoIdx])
				{
					listen[servoIdx] |= 2;
					break;
				}
			}
		}
		break;

	case 1: // ignore(id)
		chainAddress = 1023;
		if (argument == 256)
		{
			// Listen all
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				listen[servoIdx] = 0;
			}
		}
		else
		{
			// Listen Specific
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				if (argument == servoConfig[servoIdx])
				{
					listen[servoIdx] = 2;
					break;
				}
			}
		}
		break;

	case 2: // listen to only the next command
		chainAddress = 1023;
		if (argument == 256)
		{
			// Listen all
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				listen[servoIdx] |= 1;
			}
		}
		else if (argument >= 512)
		{
			//Update chainAddress
			chainAddress = argument - 512;
			// Then set listen based on chainAddress
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				if (chainAddress == servoConfig[servoIdx])
				{
					listen[servoIdx] |= 1;
					break;
				}
			}
		}
		else
		{
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				if (argument == servoConfig[servoIdx])
				{
					listen[servoIdx] |= 1;
					break;
				}
			}
		}
		break;

		//       0 enguage cached position                              {always obey command}
		//       1 turn servo on                                        {obey if listening}
		//       2 turn servo off                                       {obey if listening}
		//       3 set cmdpos to curpos                                 {obey if listening}

	case 3: // set flags
		debugToggle();

		if (IsHigh(ENGCACHE, argument))
		{
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				cmdpos[servoIdx] = cachepos[servoIdx];
			}
		}

		if (IsHigh(CMD2CUR, argument))
		{
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				if (listen[servoIdx])
				{
					cmdpos[servoIdx] = 0xFF; // Rue originally read the ADC values
				}
			}
		}

		if (IsHigh(SERVOON, argument))
		{
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				if (listen[servoIdx])
				{
					// Turn on
				}
			}
		}
		else if (IsHigh(SERVOOFF, argument))
		{
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				if (listen[servoIdx])
				{
					// Turn Off
				}
			}
		}
		break;

	case 4: // set servo position
		for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
		{
			if (listen[servoIdx])
			{
				cmdpos[servoIdx] = argument;
			}
		}
		break;

	case 5: // set cached position
		for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
		{
			if (listen[servoIdx])
			{
				cachepos[servoIdx] = argument;
			}
		}
		break;

	case 6: // get servo current
		for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
		{
			if (argument == servoConfig[servoIdx])
			{
				reply = PackBits(0, ReplyCur);
				break;
			}
		}
		break;

	case 7: // get servo position
		for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
		{
			if (argument == servoConfig[servoIdx])
			{
				reply = PackBits(1, ReplyCur);
				break;
			}
		}
		break;

	case 8: // get model
		for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
		{
			if (argument == servoConfig[servoIdx])
			{
				reply = PackBits(Version, ReplyCur);
				break;
			}
		}
		break;
	}

	switch (command)
	{ // clear one time flags
	case 3:
	case 4:
	case 5:
		for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
		{
			listen[servoIdx] &= 2;
		}
		if (chainAddress != 1023)
		{
			chainAddress++;
			for (uint8_t servoIdx = 0; servoIdx < numServos; servoIdx++)
			{
				if (chainAddress == servoConfig[servoIdx])
				{
					listen[servoIdx] = 1;
					break;
				}
			}
		}
		break;
	}

	return reply;
}