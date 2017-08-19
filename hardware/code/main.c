#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/i2c.h>

#include <stdint.h>





static void clock_setup(void);
static void gpio_setup(void);







static void clock_setup(void)
{
	//rcc_clock_setup_in_hse_16mhz_out_72mhz();

	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);
	//I2C
	rcc_periph_clock_enable(RCC_I2C2);
	rcc_periph_clock_enable(RCC_AFIO);
}


static void gpio_setup(void)
{
	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); //PC13 is the LED
			  
	//setup i2c pins
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, 
				  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
				  GPIO10 | GPIO11); //B10 =SCL, B11=SDA
		//		  GPIO6 | GPIO7);

}

static void i2c_setup(void){
	
	//disable i2c before changing config
	i2c_peripheral_disable(I2C2);
	i2c_reset(I2C2);
	
	i2c_set_standard_mode(I2C2);
	i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_34MHZ); //half system clock
	
	i2c_set_ccr(I2C2, 4000);
	i2c_set_dutycycle(I2C2, I2C_CCR_DUTY_DIV2);
	i2c_enable_ack(I2C2);
	
	
	//enable it
	i2c_peripheral_enable(I2C2);
	
}


static void i2cSend(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t data[], uint8_t numData){
	
 	uint32_t reg32 __attribute__((unused));
	
	reg32 = 3;
	if (reg32 >= 3) {
		reg32 = 5;
	}
	
	//send start
	i2c_send_start(i2c);
	
	if (reg32>=5){
		reg32 = 8;
	}
	
	//wait for the switch to master mode.
	while (!((I2C_SR1(i2c) & I2C_SR1_SB) & 
			 (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
			
	//send address
	i2c_send_7bit_address(i2c, addr, I2C_WRITE);
	
	//cleaning ADDR condition sequence
	//	have to read here for reasons?
	reg32 = I2C_SR2(i2c); 
	
	i2c_send_data(i2c, reg);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF)); //wait for byte transferred flag
	
	
	for (int i=0;i<numData;i++) {
		i2c_send_data(i2c, data[i]);
		while (!(I2C_SR1(i2c) & I2C_SR1_BTF)); //wait for byte transferred flag
	}
	
	
	//send stop condition
	i2c_send_stop(i2c); 
	
}



int main(void) {

	
	clock_setup();
	gpio_setup();
	i2c_setup();
	
	
	uint32_t i2c2 = I2C2; //i2c2
	
	uint8_t address = 0x80 >> 1; //loc3 bitshifts for us
	uint8_t data[4];
	
	uint8_t singleData[1];
	
	//set the pwm frequency
	singleData[0] = 0x04;
	i2cSend(i2c2, address, 0x00, singleData, 1); //mode1 -> sleep mode
	singleData[0] = 0x70;
	i2cSend(i2c2, address, 0xFE, singleData, 1); //prescale reg -> 60hz????
	singleData[0] = 0x20 | 0x01;
	i2cSend(i2c2, address, 0x00, singleData, 1);//mode1 -> auto increment | allcall
	
	uint16_t servoLow = 0x30;
	uint16_t servoHigh = 0x30 + 0xFF;
	
	uint8_t l1;
	uint8_t l2;
	uint8_t h1;
	uint8_t h2;
	
	l1 = (uint8_t) servoLow;
	l2 = (uint8_t) (servoLow>>8);
	h1 = (uint8_t) servoHigh;
	h2 = (uint8_t) (servoHigh>>8);
	
	//servo_init();

	while (1) {
		
		//servo_set_position(SERVO_CH1, SERVO_MAX);
		

		
		data[0] = l1;
		data[1] = l2;
		data[2] = h1;
		data[3] = h2;
		
		//i2cSend(i2c2, address, 0x06+4*0, data, 4);
		
		for (int i=0; i<900000; i++){
			__asm__("nop");
		}
		gpio_set(GPIOC, GPIO13);
		
		//servo_set_position(SERVO_CH1, SERVO_MIN);
		for (int i=0; i<900000; i++){
			__asm__("nop");
		}
		gpio_clear(GPIOC, GPIO13);
		
		data[0] = h1;
		data[1] = h2;
		data[2] = l1;
		data[3] = l2;
		//i2cSend(i2c2, address, 0x06+4*0, data, 4);

	}

	return 0;
}
