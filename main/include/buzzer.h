#ifndef _BUZZER
#define _BUZZER
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
    int buzzer_pin; 
} buzzer_t;

void buzzer_init(buzzer_t *buzzer);
void buzzer_on(buzzer_t *buzzer);
void buzzer_off(buzzer_t *buzzer);
void buzzer_blink(buzzer_t *buzzer, int high_interval_ms, int low_interval_ms);


#endif