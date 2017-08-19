#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <stdint.h>

#include "lowlevel/servo.h"



static void clock_setup(void);
static void gpio_setup(void);







static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOC);
}


static void gpio_setup(void)
{
	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); //PC13 is the LED
}



int main(void) {

	
	clock_setup();
	gpio_setup();
	
	servo_init();

	while (1) {
		
		servo_set_position(SERVO_CH1, SERVO_MAX);
		for (int i=0; i<9000000; i++){
			__asm__("nop");
		}
		gpio_toggle(GPIOC, GPIO13);
		
		servo_set_position(SERVO_CH1, SERVO_MIN);
		for (int i=0; i<9000000; i++){
			__asm__("nop");
		}

	}

	return 0;
}
