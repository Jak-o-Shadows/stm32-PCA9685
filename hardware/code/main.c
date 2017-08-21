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
	i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_16MHZ); 
	
	i2c_set_ccr(I2C2, 0xAA);
	i2c_set_dutycycle(I2C2, I2C_CCR_DUTY_DIV2);
	
	i2c_set_trise(I2C2, 0x11);
	
	i2c_enable_ack(I2C2);
	
	
	//enable it
	i2c_peripheral_enable(I2C2);
	
}


static void i2cSend(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t data[], uint8_t numData){
	
 	uint32_t reg32 __attribute__((unused));
	
	//send start
	i2c_send_start(i2c);
		
	//wait for the switch to master mode.
	while (!((I2C_SR1(i2c) & I2C_SR1_SB) & 
			 (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
			
	//send address
	i2c_send_7bit_address(i2c, addr, I2C_WRITE);
	//check for the ack
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	//must read SR2 to clear it
	reg32 = I2C_SR2(i2c);
	
	
	i2c_send_data(i2c, reg);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF)); //wait for byte transferred flag
	
	
	for (int i=0;i<numData;i++) {
		i2c_send_data(i2c, data[i]);
		while (!(I2C_SR1(i2c) & I2C_SR1_BTF)); //wait for byte transferred flag
	}
	
	
	//send stop condition
	i2c_send_stop(i2c); 
	for (int i=0;i<200;i++){
		__asm__("nop");
	}
	
}


uint16_t servoValue(uint8_t servoPos){
	//for a max/min, find the pwm length
	//working in uS, find the number of steps for the x, y ms length pulses
	// *0.9 due to inaccuracy in the frequency (55hz) used
	//		experimentally found
	uint32_t servoLow = (0.4*1000)/((1000000*1/54/4096))*0.9;
	uint32_t servoHigh =(2.7*1000)/((1000000*1/54/4096))*0.9;

	uint8_t maxServo = 180;
	uint8_t minServo = 0;
	
	uint32_t proportion = (maxServo-servoPos)*(servoHigh-servoLow)/(maxServo-minServo) + servoLow;
	
	uint16_t retVal = (uint16_t) proportion;
	
	return retVal;
		
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
	singleData[0] = 0x10;
	i2cSend(i2c2, address, 0x00, singleData, 1); //mode1 -> sleep mode
	singleData[0] = 0x70;
	i2cSend(i2c2, address, 0xFE, singleData, 1); //prescale reg -> 60hz????
	singleData[0] = 0x20 | 0x01 | 0x80; //allcall, auto increment, restart
	i2cSend(i2c2, address, 0x00, singleData, 1);//mode1 -> auto increment | allcall | restart
	
	
	
	//working in uS, find the number of steps for the x, y ms length pulses
	// *0.9 due to inaccuracy in the frequency (55hz) used
	//		experimentally found
	uint16_t servoLow = (0.4*1000)/((1000000*1/54/4096))*0.9;
	uint16_t servoHigh =(2.7*1000)/((1000000*1/54/4096))*0.9;
	
	uint16_t servoPos = servoValue(90);
	
	uint8_t pos1[4];
	uint8_t pos2[4];
	pos1[1] = 0x00;
	pos1[0] = 0x30;
	pos1[2] = (uint8_t) (0x30+servoLow);
	pos1[3] = (uint8_t) ((0x30+servoLow) >> 8);
	
	pos2[1] = 0x00;
	pos2[0] = 0x30;
	//pos2[2] = (uint8_t) (0x30+servoHigh);
	//pos2[3] = (uint8_t) ((0x30+servoHigh)>>8);
	pos2[2] = (uint8_t) (0x30+servoPos);
	pos2[3] = (uint8_t) ((0x30+servoPos)>>8);
	
	

	while (1) {
		
		
		i2cSend(i2c2, address, 0x06+4*0, pos1, 4);

		
		for (int i=0; i<2400000; i++){
			__asm__("nop");
		}
		gpio_set(GPIOC, GPIO13);
		
		i2cSend(i2c2, address, 0x06+4*0, pos2, 4);

		
		for (int i=0; i<2400000; i++){
			__asm__("nop");
		}
		gpio_clear(GPIOC, GPIO13);
		


	}

	return 0;
}
