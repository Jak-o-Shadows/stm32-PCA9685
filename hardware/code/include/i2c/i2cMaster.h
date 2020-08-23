#ifndef I2CMASTERFILE
#define I2CMASTERFILE

#include <stdint.h>

void i2cMaster_setup(uint32_t i2c);

void i2cMaster_send(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t data[], uint8_t numData);

#endif
