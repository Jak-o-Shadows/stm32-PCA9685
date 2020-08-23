#include "i2c/i2cMaster.h"

#include <libopencm3/stm32/rcc.h> // Must include to fix 'BEGIN_DECLS' does not name a type

#include <libopencm3/stm32/f1/i2c.h>

void i2cMaster_setup(uint32_t i2c)
{
    //disable i2c before changing config
    i2c_peripheral_disable(i2c);
    i2c_reset(i2c);

    i2c_set_standard_mode(i2c);
    i2c_set_clock_frequency(i2c, I2C_CR2_FREQ_16MHZ);

    i2c_set_ccr(i2c, 0xAA);
    i2c_set_dutycycle(i2c, I2C_CCR_DUTY_DIV2);

    i2c_set_trise(i2c, 0x11);

    i2c_enable_ack(i2c);

    //enable it
    i2c_peripheral_enable(i2c);
}

void i2cMaster_send(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t data[], uint8_t numData)
{
    uint32_t reg32 __attribute__((unused));

    //send start
    i2c_send_start(i2c);

    //wait for the switch to master mode.
    while (!((I2C_SR1(i2c) & I2C_SR1_SB) &
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
        ;

    //send address
    i2c_send_7bit_address(i2c, addr, I2C_WRITE);
    //check for the ack
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR))
        ;
    //must read SR2 to clear it
    reg32 = I2C_SR2(i2c);

    i2c_send_data(i2c, reg);
    while (!(I2C_SR1(i2c) & I2C_SR1_BTF))
        ; //wait for byte transferred flag

    for (int i = 0; i < numData; i++)
    {
        i2c_send_data(i2c, data[i]);
        while (!(I2C_SR1(i2c) & I2C_SR1_BTF))
            ; //wait for byte transferred flag
    }

    //send stop condition
    i2c_send_stop(i2c);
    for (int i = 0; i < 200; i++)
    {
        __asm__("nop");
    }
}
