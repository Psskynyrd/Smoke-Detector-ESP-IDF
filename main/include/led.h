#ifndef __LED__
#define __LED__
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
    int led_pin;
} led_t;

void led_init(led_t *led);
void led_open(led_t *led);
void led_off(led_t *led);
void led_blink(led_t *led, int high_interval_ms, int low_interval_ms);

#endif