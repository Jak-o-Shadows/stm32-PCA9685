

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/i2c.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/cm3/nvic.h>

#include <stdint.h>


#define NUMCHANNELS 16

#include "main.h"

uint16_t mode = 0;

uint32_t timeCount = 0;
uint32_t maxTimeCount = 300;
uint32_t key[] = {100, 0, 200};
uint16_t val[] = {0xFFFF, 0xFF, 0x1FF};
uint16_t oldVal = 0x00;
uint32_t numElements = 3;





void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOA clock (for LED GPIOs). */
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART1);
	
	rcc_periph_clock_enable(RCC_GPIOB);
	//I2C
	rcc_periph_clock_enable(RCC_I2C2);
	
	
}


void gpio_setup(void)
{
	gpio_set(GPIOC, GPIO13);

	/* Setup GPIO for LED use. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
			  
	//setup i2c pins
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, 
				  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
				  GPIO10 | GPIO11); //B10 =SCL, B11=SDA
		//		  GPIO6 | GPIO7);
}


void timer_setup(void){
	
	rcc_periph_clock_enable(RCC_TIM2);
	
	nvic_enable_irq(NVIC_TIM2_IRQ);
	
	rcc_periph_reset_pulse(RST_TIM2);
	
	//Timer global mode:
	//	no divider
	//	alignmetn edge
	//	direction up
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	
	//see loc3 examples, stm32-h103/timer/timer.c
	timer_set_prescaler(TIM2, 1); //was 250
	
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);
	
	timer_set_period(TIM2, 65535);
	
	timer_set_oc_value(TIM2, TIM_OC1, 1000); //was 10000
	
	timer_enable_counter(TIM2);
	
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}


/////////////////////////////////////////////////////////
////////// PCA9685 Servo Stuff //////////////////////////
/////////////////////////////////////////////////////////

void i2c_setup(void){
	
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



void i2cSend(uint32_t i2c, uint8_t addr, uint8_t reg, uint8_t data[], uint8_t numData){
	
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



void setServoPos(uint8_t servo, uint16_t pos){
	//pos is the pwm value given to the pwm controller
	uint8_t data[4];
	data[1] = 0x00;
	data[0] = 0x30;
	data[2] = (uint8_t) (0x30+pos);
	data[3] = (uint8_t) ((0x30+pos) >> 8);
	i2cSend(I2C2, 0x80 >> 1, 0x06+4*servo, data, 4);
}





////////////////////////////////////////////////



int main(void)
{
	clock_setup();
	gpio_setup();
	i2c_setup();
	
	
	uint32_t i2c2 = I2C2; //i2c2
	
	uint8_t address = 0x80 >> 1; //loc3 bitshifts for us
	
	uint8_t singleData[1];
	
	//set the pwm frequency
	singleData[0] = 0x10;
	i2cSend(i2c2, address, 0x00, singleData, 1); //mode1 -> sleep mode
	singleData[0] = 0x70;
	i2cSend(i2c2, address, 0xFE, singleData, 1); //prescale reg -> 60hz????
	singleData[0] = 0x20 | 0x01 | 0x80; //allcall, auto increment, restart
	i2cSend(i2c2, address, 0x00, singleData, 1);//mode1 -> auto increment | allcall | restart */
	
	//Initialise to full on
	for (int i=0; i<NUMCHANNELS; i++){
		setServoPos(i, 0xFFFF);
	}
	
	timer_setup();


	/* Wait forever and do nothing. */
	while (1) {
/* 		for (int i=0;i<1000000;i++){
			__asm__("nop");
		}
		for (int i=0; i<NUMCHANNELS; i++){
			setServoPos(i, 0x00);
		}
		for (int i=0;i<1000000;i++){
			__asm__("nop");
		}
		for (int i=0; i<NUMCHANNELS; i++){
			setServoPos(i, 0xFFFF);
		} */
	}
	return 0;
}

/////////////////////////////////////////////////
// ////////////////// INTERRUPTS ///////////////
/////////////////////////////////////////////


void tim2_isr(void) {
	if (timer_get_flag(TIM2, TIM_SR_CC1IF)){
		timer_clear_flag(TIM2, TIM_SR_CC1IF);
		
		//uint16_t counter = timer_get_counter(TIM2);
		
		timeCount++;
		
		uint32_t node = 1;
		uint32_t oldNode = 1;
		uint32_t parentNode = 2;
		uint32_t newNode;
		while (node<=numElements){
			if (timeCount > key[node-1]){
				parentNode = node;
				oldNode = node;
				node = 2*node + 1;
			} else if (timeCount < key[node-1]){
				newNode = 2*node;
				if (newNode >= numElements){
					oldNode = parentNode;
					break;
				}
				parentNode = oldNode;
				oldNode = node;
				node = newNode;
			} else {
				oldNode = node;
				break;
			}
		}
		uint16_t v = val[oldNode-1];
		if (v != oldVal){
		//	for (int i=1; i<NUMCHANNELS; i++){
		//		setServoPos(i, v);
		//	}
			setServoPos(14, oldVal);
			setServoPos(15, v);
			oldVal = v;
		}
		
		if (timeCount>maxTimeCount){
			timeCount = 0;
		}
		
		
/* 		if (mode == 0) {
			for (int i=0; i<NUMCHANNELS; i++){
				setServoPos(i, 0xFFFF);
			}
			mode = 1;
		} else {
			for (int i=0; i<NUMCHANNELS; i++){
				setServoPos(i, 0x00);
			}
			mode = 0;
		} */
		
		gpio_toggle(GPIOC, GPIO13);
		
		
	}
}




