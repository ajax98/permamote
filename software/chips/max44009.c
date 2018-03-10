#include <math.h>

#include "max44009.h"

static nrf_drv_twi_t* twi_instance;

static inline void max44009_reg_read(uint8_t reg, uint8_t* data, size_t len) {
  nrf_drv_twi_enable(twi_instance);
  nrf_drv_twi_tx(twi_instance, MAX44009_ADDR, &reg, 1, true);
  nrf_drv_twi_rx(twi_instance, MAX44009_ADDR, data, len);
  nrf_drv_twi_disable(twi_instance);
}

static inline void max44009_reg_write(uint8_t reg, uint8_t data) {
  uint8_t cmd[2] = {reg, data};

  nrf_drv_twi_enable(twi_instance);
  nrf_drv_twi_tx(twi_instance, MAX44009_ADDR, cmd, 2, true);
  nrf_drv_twi_disable(twi_instance);
}

void max44009_init(nrf_drv_twi_t* instance) {
  twi_instance = instance;
}

void max44009_config(max44009_config_t config) {
  uint8_t config_byte = config.continuous << 7 |
                        config.manual << 6 |
                        config.cdr << 3 |
                        (config.int_time & 0x7);

  max44009_reg_write(MAX44009_CONFIG, config_byte);
}

float max44009_read_lux(void) {
  uint8_t exp, mantissa;
  uint8_t data[2];
  max44009_reg_read(MAX44009_LUX_HIGH, (uint8_t*)&data, 2);
  exp = (data[0] & 0xF0) >> 4;
  mantissa = data[0] & 0xF0;
  mantissa |= data[1] & 0xF;
  return pow(2, exp) * mantissa * 0.045;
}
