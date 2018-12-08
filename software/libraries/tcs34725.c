#include <math.h>

#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"

#include "permamote.h"
#include "tcs34725.h"

static const nrf_twi_mngr_t* twi_mngr_instance;
static tcs34725_config_t tcs_config = {
  .int_time = TCS34725_INTEGRATIONTIME_2_4MS,
  .gain     = TCS34725_GAIN_1X
};

static const tcs34725_agc_t agc_list[] = {
  { {TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_700MS},     0, 20000 },
  { {TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_154MS},  4990, 63000 },
  { {TCS34725_GAIN_16X, TCS34725_INTEGRATIONTIME_154MS}, 16790, 63000 },
  { {TCS34725_GAIN_4X,  TCS34725_INTEGRATIONTIME_154MS}, 15740, 63000 },
  { {TCS34725_GAIN_1X,  TCS34725_INTEGRATIONTIME_154MS}, 15740, 0 }
};

static uint8_t agc_cur = 0;
static uint16_t atime_ms = 0;

void tcs34725_init(const nrf_twi_mngr_t* instance) {
  twi_mngr_instance = instance;
  nrf_gpio_cfg_output(TCS34725_EN);
  tcs34725_off();
}

void  tcs34725_on(void){
  nrf_gpio_pin_clear(TCS34725_EN);
  nrf_delay_ms(3);
}
void  tcs34725_off(void){
  nrf_gpio_pin_set(TCS34725_EN);
}

void tcs34725_config_agc() {
 tcs34725_config(agc_list[agc_cur].config);
}

void tcs34725_config(tcs34725_config_t config) {
  // set gain and integration time
  uint8_t int_time_reg[2] = {TCS34725_COMMAND_BIT | TCS34725_ATIME, 0};
  uint8_t gain_reg[2] = {TCS34725_COMMAND_BIT | TCS34725_CONTROL, 0};
  tcs_config = config;
  atime_ms = (uint16_t)(256 - tcs_config.int_time) * 2.4 * 1.5;
  int_time_reg[1] = tcs_config.int_time;
  gain_reg[1] = tcs_config.gain;

  nrf_twi_mngr_transfer_t const config_transfer[] = {
    NRF_TWI_MNGR_WRITE(TCS34725_ADDRESS, int_time_reg, 2, 0),
    NRF_TWI_MNGR_WRITE(TCS34725_ADDRESS, gain_reg, 2, 0),
  };
  int error = nrf_twi_mngr_perform(twi_mngr_instance, NULL, config_transfer, 2, NULL);
  APP_ERROR_CHECK(error);
}

void tcs34725_enable(void) {
  uint8_t reg[2] = {TCS34725_COMMAND_BIT | TCS34725_ENABLE, TCS34725_ENABLE_PON};

  nrf_twi_mngr_transfer_t const enable_transfer[] = {
    NRF_TWI_MNGR_WRITE(TCS34725_ADDRESS, reg, 2, 0),
  };

  int error = nrf_twi_mngr_perform(twi_mngr_instance, NULL, enable_transfer, 1, NULL);
  APP_ERROR_CHECK(error);
  reg[1] = reg[1] | TCS34725_ENABLE_AEN;
  error = nrf_twi_mngr_perform(twi_mngr_instance, NULL, enable_transfer, 1, NULL);
  APP_ERROR_CHECK(error);
  nrf_delay_ms(3);
}

void  tcs34725_disable(void){
  uint8_t reg[2] = {TCS34725_COMMAND_BIT | TCS34725_ENABLE, 0};

  nrf_twi_mngr_transfer_t const enable_read_transfer[] = {
    NRF_TWI_MNGR_WRITE(TCS34725_ADDRESS, reg, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ( TCS34725_ADDRESS, reg+1, 1, 0),
  };
  nrf_twi_mngr_transfer_t const enable_write_transfer[] = {
    NRF_TWI_MNGR_WRITE(TCS34725_ADDRESS, reg, 2, 0),
  };

  int error = nrf_twi_mngr_perform(twi_mngr_instance, NULL, enable_read_transfer, 2, NULL);
  APP_ERROR_CHECK(error);

  reg[1] &= ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);

  error = nrf_twi_mngr_perform(twi_mngr_instance, NULL, enable_write_transfer, 1, NULL);
  APP_ERROR_CHECK(error);
}

void  tcs34725_read_channels_agc(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
  uint16_t red,green,blue,clear = 0;
  while(1) {
    tcs34725_read_channels(&red, &green, &blue, &clear);
    if (agc_list[agc_cur].maxcnt && clear > agc_list[agc_cur].maxcnt) {
        agc_cur++;
    } else if (agc_list[agc_cur].mincnt && clear < agc_list[agc_cur].mincnt) {
        agc_cur--;
    }
    else break;
    tcs34725_disable();
    tcs34725_enable();
    tcs34725_config(agc_list[agc_cur].config);
  }
  *r = red;
  *g = green;
  *b = blue;
  *c = clear;
}

void  tcs34725_read_channels(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
  uint8_t reg[1] = {TCS34725_COMMAND_BIT | TCS34725_CDATAL};
  uint16_t data[4] = {0};
  nrf_twi_mngr_transfer_t const channel_read_transfer[] = {
    NRF_TWI_MNGR_WRITE(TCS34725_ADDRESS, reg, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ( TCS34725_ADDRESS, (uint8_t*) data, 8, 0),
  };

  nrf_delay_ms(atime_ms);

  int error = nrf_twi_mngr_perform(twi_mngr_instance, NULL, channel_read_transfer, sizeof(channel_read_transfer)/sizeof(channel_read_transfer[0]), NULL);
  APP_ERROR_CHECK(error);

  *c = data[0];
  *r = data[1];
  *b = data[2];
  *g = data[3];

}

void tcs34725_ir_compensate(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
  uint32_t IR = (*r + *g + *b > *c) ? (*r + *g + *b - *c) / 2 : 0;
  *r -= IR;
  *g -= IR;
  *b -= IR;
  *c -= IR;
}

float tcs34725_calculate_ct(uint16_t r, uint16_t b) {
  return (3810*(float)b) / (float)r + 1391;
}

float tcs34725_calculate_cct(uint16_t r, uint16_t g, uint16_t b) {
  // Based on: https://ams.com/documents/20143/80162/TCS34xx_AN000517_1-00.pdf/1efe49f7-4f92-ba88-ca7c-5121691daff7
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return cct;
}

float tcs34725_calculate_lux(uint16_t r, uint16_t g, uint16_t b) {

  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  // Unclear whether this works
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return illuminance;
}
