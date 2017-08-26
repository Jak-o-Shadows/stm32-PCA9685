#ifndef MAINFILE
#define MAINFILE


void clock_setup(void);
void gpio_setup(void);

void i2c_setup(void);
void i2cSend(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t data[], uint8_t numData);


uint16_t servoValue(uint8_t servoPos);
void setServoPos(uint8_t servo, uint16_t pos);
uint16_t widthToPos(uint8_t width);



void handlePacket(uint8_t pckt[]);










#endif