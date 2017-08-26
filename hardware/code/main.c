

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/i2c.h>

#include <libopencm3/cm3/nvic.h>

#include <stdint.h>


#define NUMSERVOS 16

#include "main.h"

uint16_t positionCache[32];
uint16_t position[32];

//comms side stuff
uint8_t rxBuf[4];
uint8_t rxCount = 0;


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

void usart_setup(void)
{
	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);

	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port B for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 9600);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);

	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART1);
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


uint16_t servoValue(uint8_t servoPos){
	//Convert an angle (int) to servo pulse length
	
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

void setServoPos(uint8_t servo, uint16_t pos){
	//pos is the pwm value given to the pwm controller
	uint8_t data[4];
	data[1] = 0x00;
	data[0] = 0x30;
	data[2] = (uint8_t) (0x30+pos);
	data[3] = (uint8_t) ((0x30+pos) >> 8);
	i2cSend(I2C2, 0x80 >> 1, 0x06+4*servo, data, 4);
}

uint16_t widthToPos(uint8_t width){
	//Convert a 0-0xFF number to a servo pulse length
	
	//for a max/min, find the pwm length
	//working in uS, find the number of steps for the x, y ms length pulses
	// *0.9 due to inaccuracy in the frequency (55hz) used
	//		experimentally found
	uint32_t servoLow = (0.4*1000)/((1000000*1/54/4096))*0.9;
	uint32_t servoHigh =(2.7*1000)/((1000000*1/54/4096))*0.9;

	uint8_t maxServo = 0xFF;
	uint8_t minServo = 0;
	
	uint32_t proportion = (maxServo-width)*(servoHigh-servoLow)/(maxServo-minServo) + servoLow;
	
	uint16_t retVal = (uint16_t) proportion;

	return retVal;
}

/////////////////////////////////////////////////////
////////////// Comms Stuff //////////////////////////
/////////////////////////////////////////////////////



void handlePacket(uint8_t pckt[]) {
	
	if (pckt[0] == 0xFF) {
		uint8_t pos = pckt[2];
		if (pckt[1] < 32 ) {
			//immediate move
			position[pckt[1]] = widthToPos(pos);
			setServoPos(pckt[1], position[pckt[1]]);
		} else if (pckt[1] < 64) {
			//cache position
			positionCache[pckt[1]-32] = widthToPos(pos);
		} else if (pckt[1] == 0xFF){
			//apply cached position
			for (int i=0; i<NUMSERVOS; i++){
				setServoPos(i, positionCache[i]);
			}
		} 
	} else {
		__asm__("nop"); //woah, out of sync somehow
	}
	
	
}


////////////////////////////////////////////////



int main(void)
{
	clock_setup();
	gpio_setup();
	usart_setup();
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
	
	
	
	

	/* Wait forever and do nothing. */
	while (1)
		__asm__("nop");

	return 0;
}


void usart1_isr(void)
{
	static uint8_t data = 'A';

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOC, GPIO13);

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART1);
		rxBuf[rxCount] = data;
		rxCount++;
		if (rxCount>=3) {
			rxCount = 0;
			handlePacket(rxBuf);
		}

		/* Enable transmit interrupt so it sends back the data. */
		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		/* Indicate that we are sending out data. */
		// gpio_toggle(GPIOA, GPIO7);

		/* Put data into the transmit register. */
		usart_send(USART1, data);

		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(USART1) &= ~USART_CR1_TXEIE;
	}
}


