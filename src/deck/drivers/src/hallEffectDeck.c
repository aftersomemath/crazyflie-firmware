// #include <stdint.h>
// #include <stdlib.h>
#include "FreeRTOS.h"
#include "stm32fxxx.h"
#include "semphr.h"
// #include "config.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "timers.h"
#include "hallEffectDeck.h"

#include "as5600.h"
#include "i2cdev.h"

#define ANALOG_PIN_3 DECK_GPIO_MISO
#define ANALOG_PIN_4 DECK_GPIO_MOSI




// static uint32_t sensor_0_cc = 0, sensor_1_cc = 0;
static uint32_t ms_cc = 0;
static uint8_t sensor_1_data[10];
static uint8_t  sensor_0_data[10];
static float l_bound=1.45, u_bound=2.15;
static float sensor_0_volts;
static float sensor_1_volts;
static xTimerHandle timerHndl1ms;

static SemaphoreHandle_t mutex;

// I2C_Dev AS5600_i2c_device;

uint32_t const my_i2c_xfer(uint8_t const slave_address,
                           uint8_t const * const p_tx_buffer,
                           size_t const tx_buffer_size,
                           uint8_t * const p_rx_buffer,
                           size_t const rx_buffer_size)
{

  bool success_bool = true;
  bool is_rx_operation = true;
  
  
  if ((NULL == p_tx_buffer) || (0 == tx_buffer_size)) {
    success_bool = false;

  } else if ((NULL == p_rx_buffer) || (0 == rx_buffer_size)) {
    is_rx_operation = false;
  }

  if (success_bool) {
    // TX operation
    success_bool = i2cdevWrite(I2C1_DEV, slave_address, tx_buffer_size, p_tx_buffer);
  }
  
  if ((success_bool) && (is_rx_operation)) {
    // RX operation
    success_bool = i2cdevRead(I2C1_DEV, slave_address, rx_buffer_size, p_rx_buffer);
  }
  
  return success_bool;
}


static uint8_t float_pack(float in){
  if (in<u_bound)
    return (uint8_t) (((in-l_bound)/(u_bound-l_bound))*255);
  return (uint8_t) 255;
}

static void flap_ms_timer(xTimerHandle pxTimer) {
  
  sensor_0_volts = analogReadVoltage(ANALOG_PIN_3);
  sensor_1_volts = analogReadVoltage(ANALOG_PIN_4);
  // if (ms_cc%10 == 0)
  //   xSemaphoreTake(mutex, portTICK_PERIOD_MS);
  sensor_0_data[ms_cc%10] = float_pack(sensor_0_volts);
  // if (ms_cc%10 == 9)
  //   xSemaphoreGive(mutex);
  sensor_1_data[ms_cc%10] = float_pack(sensor_1_volts);
  // sensor_1_data = float_pack(sensor_1_volts);

  ms_cc++;
}


/* Main initialization */
static void hallEffectDeckInit(DeckInfo *info)
{
  // AS5600_i2c_device.I2Cx = I2Cx;/////////////////////////////////////// TODO: fix this look at vl53iiox.c
  // as5600_error_t result = as5600_init(my_i2c_xfer);
  // as5600_init(my_i2c_xfer);

  mutex = xSemaphoreCreateMutex();

  timerHndl1ms = xTimerCreate("flap_timer",pdMS_TO_TICKS(1),pdTRUE,NULL,flap_ms_timer);
  xTimerStart( timerHndl1ms,pdMS_TO_TICKS(100));

}

float loggerFuncSensor0(uint32_t timestamp, void* data){
  return analogReadVoltage(ANALOG_PIN_3);
}

float loggerFuncSensor1(uint32_t timestamp, void* data){
  return analogReadVoltage(ANALOG_PIN_4);
}


uint16_t loggerFuncSensor0Full_0(uint32_t timestamp, void* data){
  return (uint32_t)sensor_0_data[0]+((uint32_t)sensor_0_data[1]<<8);
}

uint16_t loggerFuncSensor0Full_1(uint32_t timestamp, void* data){
  return (uint16_t)sensor_0_data[2]+((uint16_t)sensor_0_data[3]<<8);
}

uint16_t loggerFuncSensor0Full_2(uint32_t timestamp, void* data){
  return (uint32_t)sensor_0_data[4]+((uint32_t)sensor_0_data[5]<<8);
}

uint16_t loggerFuncSensor0Full_3(uint32_t timestamp, void* data){
  return (uint16_t)sensor_0_data[6]+((uint16_t)sensor_0_data[7]<<8);
}

uint16_t loggerFuncSensor0Full_4(uint32_t timestamp, void* data){
  return (uint16_t)sensor_0_data[8]+((uint16_t)sensor_0_data[9]<<8);
}

uint16_t loggerFuncSensor1Full_0(uint32_t timestamp, void* data){
  return (uint32_t)sensor_1_data[0]+((uint32_t)sensor_1_data[1]<<8);
}

uint16_t loggerFuncSensor1Full_1(uint32_t timestamp, void* data){
  return (uint16_t)sensor_1_data[2]+((uint16_t)sensor_1_data[3]<<8);
}

uint16_t loggerFuncSensor1Full_2(uint32_t timestamp, void* data){
  return (uint32_t)sensor_1_data[4]+((uint32_t)sensor_1_data[5]<<8);
}

uint16_t loggerFuncSensor1Full_3(uint32_t timestamp, void* data){
  return (uint16_t)sensor_1_data[6]+((uint16_t)sensor_1_data[7]<<8);
}

uint16_t loggerFuncSensor1Full_4(uint32_t timestamp, void* data){
  return (uint16_t)sensor_1_data[8]+((uint16_t)sensor_1_data[9]<<8);
}

logByFunction_t sensor0Logger = {.aquireFloat = loggerFuncSensor0, .data = 0};
logByFunction_t sensor1Logger = {.aquireFloat = loggerFuncSensor1, .data = 0};
logByFunction_t sensor0Logger_0 = {.acquireUInt16 = loggerFuncSensor0Full_0, .data = 0};
logByFunction_t sensor0Logger_1 = {.acquireUInt16 = loggerFuncSensor0Full_1, .data = 0};
logByFunction_t sensor0Logger_2 = {.acquireUInt16 = loggerFuncSensor0Full_2, .data = 0};
logByFunction_t sensor0Logger_3 = {.acquireUInt16 = loggerFuncSensor0Full_3, .data = 0};
logByFunction_t sensor0Logger_4 = {.acquireUInt16 = loggerFuncSensor0Full_4, .data = 0};
logByFunction_t sensor1Logger_0 = {.acquireUInt16 = loggerFuncSensor1Full_0, .data = 0};
logByFunction_t sensor1Logger_1 = {.acquireUInt16 = loggerFuncSensor1Full_1, .data = 0};
logByFunction_t sensor1Logger_2 = {.acquireUInt16 = loggerFuncSensor1Full_2, .data = 0};
logByFunction_t sensor1Logger_3 = {.acquireUInt16 = loggerFuncSensor1Full_3, .data = 0};
logByFunction_t sensor1Logger_4 = {.acquireUInt16 = loggerFuncSensor1Full_4, .data = 0};


LOG_GROUP_START(flapper_speed)
  LOG_ADD_BY_FUNCTION(LOG_FLOAT,sensor_0,&sensor0Logger)
  LOG_ADD_BY_FUNCTION(LOG_FLOAT,sensor_1,&sensor1Logger)
  LOG_ADD_BY_FUNCTION(LOG_UINT16,sensor_0_0,&sensor0Logger_0)
  LOG_ADD_BY_FUNCTION(LOG_UINT16,sensor_0_1,&sensor0Logger_1)
  LOG_ADD_BY_FUNCTION(LOG_UINT16,sensor_0_2,&sensor0Logger_2)
  LOG_ADD_BY_FUNCTION(LOG_UINT16,sensor_0_3,&sensor0Logger_3)
  LOG_ADD_BY_FUNCTION(LOG_UINT16,sensor_0_4,&sensor0Logger_4)
  LOG_ADD_BY_FUNCTION(LOG_UINT16,sensor_1_0,&sensor1Logger_0)
  LOG_ADD_BY_FUNCTION(LOG_UINT16,sensor_1_1,&sensor1Logger_1)
  LOG_ADD_BY_FUNCTION(LOG_UINT16,sensor_1_2,&sensor1Logger_2)
  LOG_ADD_BY_FUNCTION(LOG_UINT16,sensor_1_3,&sensor1Logger_3)
  LOG_ADD_BY_FUNCTION(LOG_UINT16,sensor_1_4,&sensor1Logger_4)
LOG_GROUP_STOP(flapper_speed)

static const DeckDriver hall_effect_deck = {
  .vid = 0,
  .pid = 0,
  .name = "hallEffectDeck",
  .usedGpio = DECK_USING_PA6 | DECK_USING_PA7,
  .init = hallEffectDeckInit,
};

DECK_DRIVER(hall_effect_deck);