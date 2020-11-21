// Display app
//
// Write messages to a Newhaven OLED display over SPI

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "pixy2_spi.h"


#define SHOW_STATUS(code, comment) printf(comment " %d\n", code)


pixy_t *pixy;
pid_t panLoop;


void setup() {
  // initialize RTT library
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  // initialize spi master
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_SD_SCLK,
    .mosi_pin = BUCKLER_SD_MOSI,
    .miso_pin = BUCKLER_SD_MISO,
    .ss_pin = BUCKLER_SD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_3,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL));

  // We need to initialize the pixy object 
  pixy2_t *pixy;
  SHOW_STATUS(pixy_init(&pixy, &spi_instance), "initialize");
  print_version(pixy->version);

  // Use color connected components program for the pan tilt to track 
  SHOW_STATUS(changeProg(pixy, PIXY_PROG_COLOR_CODE), "change program");

  pid_init(&panLoop, 400, 0, 400, true);
}


void loop() {  
  // get active blocks from Pixy
  SHOW_STATUS(getBlocks(pixy, false, CCC_SIG_ALL, CCC_MAX_BLOCKS), "blocks");
  for (int i=0; i < pixy->numBlocks; i++)
    print_block(&pixy->blocks[i]);

  SHOW_STATUS(getFPS(pixy), "FPS");

  int32_t panOffset;
  
  if (pixy->numBlocks) {            
    // calculate pan and tilt "errors" with respect to first object (blocks[0]), 
    // which is the biggest object (they are sorted by size).  
    panOffset = (int32_t)pixy->frameWidth/2 - (int32_t)pixy->blocks[0].m_x;
  
    // update loops
    pid_update(&panLoop, panOffset);
  
    // set pan and tilt servos
    kobukiDriveDirect(panLoop->m_command, -panLoop->m_command);
   
#if 0 // for debugging
    printf("%ld %ld %ld %ld", rotateLoop.m_command, translateLoop.m_command, left, right);
#endif

  // no object detected, go into reset state
  } else {
    reset(&panLoop);
    kobukiDriveDirect(0, 0);
  }
}



int main(void) {
  setup();

  while (1) {
    loop();
    nrf_delay_ms(1000);
  }
}