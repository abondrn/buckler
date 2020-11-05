// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

typedef enum {
  OFF,
  DRIVING,

  TURNING,
  SWITCHING_COURSE,
  BACKING_UP,
  TURNING_AWAY
} robot_state_t;


#define SPEED 75


char buf[16];
const float CONVERSION = 0.0006108;
static float measure_distance(uint16_t current_encoder,
                              uint16_t previous_encoder) {
  if (current_encoder < previous_encoder)
    previous_encoder = 65535 - previous_encoder;
  return CONVERSION*(current_encoder-previous_encoder);
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};

  double distance;
  uint8_t bumped;
  uint16_t previous_encoder;

  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(1);

    // handle states
    switch(state) {
      case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = DRIVING;
          previous_encoder = sensors.leftWheelEncoder;
          kobukiDriveDirect(SPEED, SPEED);
        } else {
          // perform state-specific actions here
          display_write("OFF", DISPLAY_LINE_0);
        }
        break; // each case needs to end with break!
      }

      case DRIVING: {
        // transition logic

        if (sensors.bumps_wheelDrops.bumpLeft)
            bumped = 1;
        else if (sensors.bumps_wheelDrops.bumpCenter)
            bumped = 2;
        else if (sensors.bumps_wheelDrops.bumpRight)
            bumped = 3;
        else
            bumped = 0;

        if (is_button_pressed(&sensors)) {
          distance = 0;
          state = OFF;
          kobukiDriveDirect(0, 0);
        } else if (bumped) {
          distance = 0;
          state = SWITCHING_COURSE;
          kobukiDriveDirect(-SPEED, -SPEED);
        } else if (distance >= .5) {
          distance = 0;
          state = TURNING;
          lsm9ds1_start_gyro_integration();
          kobukiDriveDirect(SPEED, -SPEED);
        } else {
          // perform state-specific actions here
          display_write("DRIVING", DISPLAY_LINE_0);

          distance += measure_distance(sensors.leftWheelEncoder, previous_encoder);
          previous_encoder = sensors.leftWheelEncoder;
          snprintf(buf, 16, "%f", distance);
          display_write(buf, DISPLAY_LINE_1);
        }
        break; // each case needs to end with break!
      }

      // add other cases here
      case TURNING: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (angle <= -90) {
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(SPEED, SPEED);
          state = DRIVING;
        } else {
          display_write("TURNING", DISPLAY_LINE_0);
          snprintf(buf, 16, "%f", angle);
          display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }

      case SWITCHING_COURSE: {
        if (measure_distance(sensors.leftWheelEncoder, previous_encoder) <= 0) {
          state = BACKING_UP;
        } else {
          previous_encoder = sensors.leftWheelEncoder;
        }
        break;
      }

      case BACKING_UP: {
        if (is_button_pressed(&sensors)) {
          distance = 0;
          state = OFF;
          kobukiDriveDirect(0, 0);
        } else if (distance <= -.1) {
          distance = 0;
          state = TURNING_AWAY;
          lsm9ds1_start_gyro_integration();
          if (bumped == 1)
            kobukiDriveDirect(SPEED, -SPEED);
          else
            kobukiDriveDirect(-SPEED, SPEED);
        } else {
          // perform state-specific actions here
          display_write("BACKING_UP", DISPLAY_LINE_0);

          distance -= measure_distance(previous_encoder, sensors.leftWheelEncoder);
          previous_encoder = sensors.leftWheelEncoder;
          snprintf(buf, 16, "%f", distance);
          display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }

      case TURNING_AWAY: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (bumped == 1 && angle <= -45 || bumped != 1 && angle >= 45) {
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(SPEED, SPEED);
          state = DRIVING;
        } else {
          display_write("TURNING_AWAY", DISPLAY_LINE_0);
          snprintf(buf, 16, "%f", angle);
          display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }
    }
  }
}

