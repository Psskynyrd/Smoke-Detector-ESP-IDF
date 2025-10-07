#include "led.h"

void led_init(led_t *led)
{
    gpio_reset_pin(led->led_pin);
    gpio_set_direction(led->led_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(led->led_pin, 0);
}

void led_open(led_t *led)
{
    gpio_set_level(led->led_pin, 1);
}

void led_off(led_t *led)
{
    gpio_set_level(led->led_pin, 0);
}

void led_blink(led_t *led, int high_interval_ms, int low_interval_ms)
{
    while (1)
    {
        gpio_set_level(led->led_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(high_interval_ms));
        gpio_set_level(led->led_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(low_interval_ms));
    }
}