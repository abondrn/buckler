// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "software_interrupt.h"

#include "gpio.h"
#include "buckler.h"


#define BUTTON0 28
#define LED0 23
#define SWITCH0 22

void SWI1_EGU1_IRQHandler(void) {
    NRF_EGU1->EVENTS_TRIGGERED[0] = 0;

    printf("1 Begin\n");
    nrf_delay_ms(5000);
    printf("1 End\n");

}

void GPIOTE_IRQHandler(void) {
    NRF_GPIOTE->EVENTS_IN[0] = 0;

    printf("2 Begin\n");
    nrf_delay_ms(1000);
    printf("2 End\n");

    //gpio_clear(LED0);
    //nrf_delay_ms(500);
    //gpio_set(LED0);
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  NRF_GPIOTE->CONFIG[0] = (BUTTON0 << 8) | (2 << 16) | 1;
  NRF_GPIOTE->INTENSET |= 1;

  NVIC_EnableIRQ(GPIOTE_IRQn);
  NVIC_SetPriority(GPIOTE_IRQn, 0);
  NVIC_SetPriority(SWI1_EGU1_IRQn, 1);

  gpio_config(BUTTON0, INPUT);
  gpio_config(LED0, OUTPUT);
  //gpio_config(SWITCH0, INPUT);
  gpio_set(LED0);

  software_interrupt_init();

  // loop forever
  while (1) {
    //if (gpio_read(SWITCH0))
    //  __WFI();
    printf("Looping\n");
    software_interrupt_generate();
    nrf_delay_ms(1000);
  }
}

