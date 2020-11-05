// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "buckler.h"
#include "gpio.h"


#define SWITCH0 22
#define LED0 23
#define BUTTON0 28

void checkoff4_2_2() {
	printf("address: %p %p\n", &GPIO->OUT, &GPIO->DIR);
 	printf("value: %lx %lx\n", GPIO->OUT, GPIO->DIR);

 	// loop forever
 	while (1);
}

void checkoff4_2_3() {
	uint32_t mask = (1<<3) - 1;
	GPIO->DIR = mask << LED0;
	for (uint32_t flips = 0; 1; flips = (flips+1)&mask) {
		GPIO->OUT = (~flips & mask) << LED0;
		nrf_delay_ms(1000);
	}
}

void checkoff4_2_4() {
	GPIO->PIN_CNF[SWITCH0].INPUT = GPIO->PIN_CNF[BUTTON0].INPUT = 0;
  	while (1) {
  		printf("switch0:%ld button0:%ld\n", (GPIO->IN >> SWITCH0) & 1, (GPIO->IN >> BUTTON0) & 1);
  		nrf_delay_ms(1000);
  	}
}

void checkoff4_2_5 () {
	gpio_config(SWITCH0, INPUT);
	gpio_config(BUTTON0, INPUT);
	gpio_config(LED0, OUTPUT);
	gpio_config(LED0+1, OUTPUT);
	 while (1) {
	 	printf("switch0:%ld button0:%ld\n", gpio_read(SWITCH0), gpio_read(BUTTON0));
  		if (gpio_read(BUTTON0)) gpio_set(LED0); else gpio_clear(LED0);
  		if (gpio_read(SWITCH0)) gpio_set(LED0+1); else gpio_clear(LED0+1);
  		nrf_delay_ms(100);
  	}
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  checkoff4_2_5();
}

