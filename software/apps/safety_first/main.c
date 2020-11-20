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


int main(void) {
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
  pixy2_t *p;
  SHOW_STATUS(init(&p, &spi_instance), "initialize");
  print_version(p->version);

  SHOW_STATUS(changeProg(p, PIXY_PROG_COLOR_CODE), "change program");

  while (1) {
    SHOW_STATUS(getBlocks(p, false, CCC_SIG_ALL, CCC_MAX_BLOCKS), "blocks");
    for (int i=0; i < p->numBlocks; i++)
      print_block(&p->blocks[i]);
    SHOW_STATUS(getFPS(p), "FPS");
    nrf_delay_ms(1000);
  }
}