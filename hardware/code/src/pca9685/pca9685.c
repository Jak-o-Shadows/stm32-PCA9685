#include "pca9685/pca9685.h"

void pca9685_setup(uint32_t i2c, uint8_t address)
{
    uint8_t addressShifted = address >> 1; //loc3 bitshifts for us

    uint8_t singleData[1];

    //set the pwm frequency
    singleData[0] = 0x10;
    i2cMaster_send(i2c, addressShifted, 0x00, singleData, 1); //mode1 -> sleep mode
    singleData[0] = 0x70;
    i2cMaster_send(i2c, addressShifted, 0xFE, singleData, 1); //prescale reg -> 60hz????
    singleData[0] = 0x20 | 0x01 | 0x80;                       //allcall, auto increment, restart
    i2cMaster_send(i2c, addressShifted, 0x00, singleData, 1); //mode1 -> auto increment | allcall | restart
}

void pca9685_setServoPos(uint32_t i2c, uint8_t address, uint8_t servo, uint16_t pos)
{
    //pos is the pwm value given to the pwm controller
    uint8_t data[4];
    data[1] = 0x00;
    data[0] = 0x30;
    data[2] = (uint8_t)(0x30 + pos);
    data[3] = (uint8_t)((0x30 + pos) >> 8);
    i2cMaster_send(i2c, address >> 1, 0x06 + 4 * servo, data, 4);
}
