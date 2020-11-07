#include "gpio.h"

volatile struct GPIO_t* const GPIO = (struct GPIO_t*)(0x50000000+0x504);

// Inputs: 
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir) {
	switch (dir) {
	case INPUT:
		GPIO->DIR &= ~(1 << gpio_num);

		struct PIN_CNF_t *pin_cnf = &GPIO->PIN_CNF[gpio_num];
    	//*pin_cnf = *pin_cnf & ~2;
		pin_cnf->INPUT = 0;
		break;
	case OUTPUT:
		GPIO->DIR |= 1 << gpio_num;
		break;
	};
}

// Set gpio_num high
// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num) {
	GPIO->OUT |= 1 << gpio_num;
}

// Set gpio_num low
// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num) {
	GPIO->OUT &= ~(1 << gpio_num);
}

// Inputs: 
//  gpio_num - gpio number 0-31
bool gpio_read(uint8_t gpio_num) {
    // should return pin state
    return (GPIO->IN >> gpio_num) & 1;
}
