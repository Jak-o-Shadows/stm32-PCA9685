#ifndef PCA9685
#define PCA9685

#include "i2c/i2cMaster.h"

#include <stdint.h>

void pca9685_setup(uint32_t i2c, uint8_t address);
void pca9685_setServoPos(uint32_t i2c, uint8_t address, uint8_t servo, uint16_t pos);

#endif
