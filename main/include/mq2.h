#ifndef _MQ2
#define _MQ2
#include <driver/gpio.h>
#include <stdio.h>
#include <string.h>
#include <rom/ets_sys.h>
#include "esp_log.h"
#include "driver/adc.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 5  // 5 = ESP_LOG_VERBOSE
#endif

typedef struct
{
    int mq2_adc_channel;
    int raw;
    float voltage;
} mq2_t;

void mq2_init(mq2_t *mq2);
int mq2_read(mq2_t *mq2);


#endif