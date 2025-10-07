#ifndef _FLAME_DETECTOR
#define _FLAME_DETECTOR
#include <driver/gpio.h>
#include <stdio.h>
#include <string.h>
#include <rom/ets_sys.h>
#include "esp_log.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 5  // 5 = ESP_LOG_VERBOSE
#endif

typedef struct
{
    int flame_sensor_pin;
    bool can_read;
    bool is_flame;
} flame_detector_t;

void flame_detector_init(flame_detector_t *flame_detector);
int flame_detector_read(flame_detector_t *flame_detector);

#endif