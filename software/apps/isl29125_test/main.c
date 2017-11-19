/*
 * Send an advertisement periodically
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "nordic_common.h"
#include "led.h"
#include "app_uart.h"

#include "permamote.h"
#include "isl29125.h"

#define MAX_TEST_DATA_BYTES  (15U)
#define UART_TX_BUF_SIZE     256
#define UART_RX_BUF_SIZE     256

nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(1);

void twi_init(void) {
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config = {
    .scl                = I2C_SCL,
    .sda                = I2C_SDA,
    .frequency          = NRF_TWI_FREQ_100K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH
  };

  err_code = nrf_drv_twi_init(&twi_instance, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);
}

void uart_error_handle (app_uart_evt_t * p_event) {
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    } else if (p_event->evt_type == APP_UART_FIFO_ERROR) {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void uart_init(void) {
  uint32_t err_code;

  const app_uart_comm_params_t comm_params =
  {
    SPI_MISO,
    SPI_MOSI,
    0,
    0,
    APP_UART_FLOW_CONTROL_DISABLED,
    false,
    UART_BAUDRATE_BAUDRATE_Baud115200
  };
  APP_UART_FIFO_INIT(&comm_params,
                     UART_RX_BUF_SIZE,
                     UART_TX_BUF_SIZE,
                     uart_error_handle,
                     APP_IRQ_PRIORITY_LOW,
                     err_code);
  APP_ERROR_CHECK(err_code);

}

int main(void) {
  // init uart
  uart_init();
  printf("\nLIGHT COLOR TEST\n");

  // Init twi
  twi_init();

  // Turn on power gate
  nrf_gpio_cfg_output(MAX44009_EN);
  nrf_gpio_cfg_output(ISL29125_EN);
  nrf_gpio_cfg_output(MS5637_EN);
  nrf_gpio_cfg_output(SI7021_EN);
  nrf_gpio_cfg_output(PIR_EN);
  nrf_gpio_pin_clear(MAX44009_EN);
  nrf_gpio_pin_clear(ISL29125_EN);
  nrf_gpio_pin_set(MS5637_EN);
  nrf_gpio_pin_set(SI7021_EN);
  nrf_gpio_pin_set(PIR_EN);

  const isl29125_config_t config = {
    .mode = isl29125_green_red_blue,
    .range=0,
    .resolution=0,
    .sync_int=0
  };

  isl29125_init(&twi_instance);
  isl29125_config(config);

  while (1) {
    float red, green, blue = 0;
    isl29125_read_lux(&red, &green, &blue);
    printf("RGB: %.02f %.02f %.02f\n", red, green, blue);
    nrf_delay_ms(5000);
  }
}
