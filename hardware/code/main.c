

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/i2c.h>

#include <libopencm3/cm3/nvic.h>

#include <stdint.h>

#include "serialServo/serialServoSlave.h"
#include "i2c/i2cMaster.h"
#include "pca9685/pca9685.h"


#include "main.h"

// Config
#define NUMSERVOS 16
#define IDOFF 0

static uint8_t SERVOADDRESSES[] = {0+ IDOFF,
								   1+ IDOFF,
								   2+ IDOFF,
								   3+ IDOFF,
								   4+ IDOFF,
								   5+ IDOFF,
								   6+ IDOFF,
								   7+ IDOFF,
								   8+ IDOFF,
								   9+ IDOFF,
								   10+IDOFF,
								   11+IDOFF,
								   12+IDOFF,
								   13+IDOFF,
								   14+IDOFF,
								   15+IDOFF,
								   16+IDOFF};

#define Version 5

#define ReplyPos 0
#define ReplyCur 1
#define ReplyVer 2



// Variables

uint16_t cachepos[NUMSERVOS];
uint16_t cmdpos[NUMSERVOS];
uint16_t _cmdpos[NUMSERVOS];
uint8_t listen[NUMSERVOS];

uint16_t updatePeriod_ms = 30;
uint16_t updateCounter_ms = 0;


void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOA clock (for LED GPIOs). */
	rcc_periph_clock_enable(RCC_GPIOC);

	rcc_periph_clock_enable(RCC_GPIOA);
	
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_GPIOA);
	
	rcc_periph_clock_enable(RCC_GPIOB);
	//I2C
	rcc_periph_clock_enable(RCC_I2C2);
	
	
}

void usart_setup(void)
{

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2); //USART 2 TX is A2
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3); //USART 2 RX is A3

	usart_set_baudrate(USART2, 9600);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	//enable interrupt rx
	USART_CR1(USART2) |= USART_CR1_RXNEIE;

	usart_enable(USART2);
}

static void nvic_setup(void)
{
	/* Without this the RTC interrupt routine will never be called. */
	nvic_enable_irq(NVIC_USART2_IRQ);
	nvic_set_priority(NVIC_USART2_IRQ, 2);
}

static void gpio_setup(void)
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


static void timer_setup(void){
	
	rcc_periph_clock_enable(RCC_TIM2);
	
	nvic_enable_irq(NVIC_TIM2_IRQ);
	
	rcc_periph_reset_pulse(RST_TIM2);
	
	//Timer global mode:
	//	no divider
	//	alignment edge
	//	direction up
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	
	// Note that TIM2 on APB1 is running at double frequency according to 
	//	https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/timer/timer.c
	timer_set_prescaler(TIM2, ((rcc_apb1_frequency  *2)/10000));
	
	// Disable preload
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);
	
	// Count the full range, as the compare value is used to set the value
	timer_set_period(TIM2, 65535);
	
	timer_set_oc_value(TIM2, TIM_OC1, 10); //was 10000
	
	timer_enable_counter(TIM2);
	
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

/////////////////////////////////////////////////////////
////////// Servo Stuff //////////////////////////
/////////////////////////////////////////////////////////

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
////////////// Main Loop   //////////////////////////
/////////////////////////////////////////////////////



int main(void)
{
	
	
	clock_setup();
	gpio_setup();
	timer_setup();
	usart_setup();
	i2cMaster_setup(I2C2);
	nvic_setup();
	
	serialServoSlave_setup(cmdpos, cachepos, listen, NUMSERVOS, SERVOADDRESSES);
	
	
	
	pca9685_setup(I2C2, 0x80);
	
	for (int i=0; i<NUMSERVOS; i++){
		// Set the hidden and commanded values to different,
		//	so that it automatically gets set.
		cmdpos[i] = 336;
		_cmdpos[i] = 0;
	}
	
	while (1) {
		__asm__("nop");
	}
}


// Interrupt Functions

void usart2_isr(void)
{
	static uint8_t data = 'A';
	static uint8_t reply = 0;

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

		// Receieve the data, using the MiniSSC protocol
		//	This protocol has a header byte (0xFF), followed
		//	by a number (0->254) followed by a number (0-254)
		data = usart_recv(USART2);
		reply = serialServoSlave_dobyte(data);
		

		/* Enable transmit interrupt so it sends back the data. */
		USART_CR1(USART2) |= USART_CR1_TXEIE;
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

		/* Indicate that we are sending out data. */
		// gpio_toggle(GPIOA, GPIO7);

		/* Put data into the transmit register. */
		usart_send(USART2, reply);

		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(USART2) &= ~USART_CR1_TXEIE;
	}
}

void tim2_isr(void)
{
	// This timer ticks every 1ms
	if (timer_get_flag(TIM2, TIM_SR_CC1IF)){
		timer_clear_flag(TIM2, TIM_SR_CC1IF);
		
		// Setup next compare time
		uint16_t compare_time = timer_get_counter(TIM2);
		timer_set_oc_value(TIM2, TIM_OC1, 10+compare_time);
		
		// Only update at a specific rate
		updateCounter_ms++;
		if( updateCounter_ms >= updatePeriod_ms ){
			// Reset counter
			updateCounter_ms = 0;
			
			// Do work
			gpio_toggle(GPIOC, GPIO13);
			
			// Set servo position if different
			for( uint8_t servoIdx=0;servoIdx<NUMSERVOS;servoIdx++ ){
				if( _cmdpos[servoIdx] != cmdpos[servoIdx] ){
					_cmdpos[servoIdx] = cmdpos[servoIdx];
					pca9685_setServoPos(I2C2, 0x80, servoIdx, _cmdpos[servoIdx]);
				}
			}
		}
	}
}
