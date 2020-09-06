#ifndef CONFIGURE_RMT
#define CONFIGURE_RMT

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "esp_log.h"

#include "PinOut.h"


// receiver pulse length ranges from 8000 to 16000 centered around 12000
#define RECEIVER_CH_MIN 8000
#define RECEIVER_CH_CENTER 12000
#define RECEIVER_CH_MAX 16000
// how many ticks +/- RECEIVER_CENTER is part of the deadzone
//#define RECEIVER_CH_DEADZONE 10

#define RMT_TICK_PER_US 8
// determines how many clock cycles one "tick" is
// [1..255], source is generally 80MHz APB clk

#define RMT_RX_CLK_DIV (80000000/RMT_TICK_PER_US/1000000)

// time before receiver goes idle
#define RMT_RX_MAX_US 3500

#define RECEIVER_CHANNELS_NUM 2

//extern volatile uint16_t ReceiverChannels[RECEIVER_CHANNELS_NUM];

void rmtInit(void);
uint16_t rmtValueRaw(int channel);
uint16_t rmtMapConstrain(int channel, int minVal, int maxVal);
uint16_t rmtValuePWM (int channel);
uint16_t rmtValueEU (int channel);
uint16_t rmtValueEUZero (int channel);

#endif
