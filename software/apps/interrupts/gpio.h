#pragma once

#include "nrf.h"
#include "stdbool.h"

typedef enum {
    INPUT = 0,
    OUTPUT,
} gpio_direction_t;

typedef unsigned int uint;
struct PIN_CNF_t {
   uint DIR      : 1;
   uint INPUT    : 1;
   uint PULL     : 2;
   uint _UNUSED1 : 4;
   uint DRIVE    : 3;
   uint _UNUSED2 : 5;
   uint SENSE    : 2;
   uint _UNUSED3 : 14;
};

struct GPIO_t {
	uint32_t OUT; // 0x504 Write GPIO port
	uint32_t OUTSET; // Set individual bits in GPIO port
	uint32_t OUTCLR; // Clear individual bits in GPIO port
	uint32_t IN; // Read GPIO port
	uint32_t DIR; // Direction of GPIO pins
	uint32_t DIRSET; // DIR set register
	uint32_t DIRCLR; // DIR clear register
	uint32_t LATCH; // Latch register indicating what GPIO pins that have met the criteria set in the PIN_CNF[n].SENSE registers
	uint32_t DETECTMODE; // Select between default DETECT signal behaviour and LDETECT mode
	uint32_t _UNUSED[118];
	struct PIN_CNF_t PIN_CNF[32];
};

extern volatile struct GPIO_t* const GPIO;


// Inputs: 
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir);

// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num);

// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num);

// Inputs: 
//  gpio_num - gpio number 0-31
// Returns:
//  current state of the specified gpio pin
bool gpio_read(uint8_t gpio_num);
