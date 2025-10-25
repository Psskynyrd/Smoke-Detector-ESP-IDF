#ifndef __MOSFET_H__
#define __MOSFET_H__
#include <driver/gpio.h>
#include <stdio.h>
#include <string.h>
#include <rom/ets_sys.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 5 // 5 = ESP_LOG_VERBOSE
#endif

typedef struct
{
    int mosfet_pin; 
} mosfet_t;

void mosfet_init(mosfet_t *mosfet);
void mosfet_on(mosfet_t *mosfet);
void mosfet_off(mosfet_t *mosfet);
void mosfet_blink(mosfet_t *mosfet, int high_interval_ms, int low_interval_ms);


#endif