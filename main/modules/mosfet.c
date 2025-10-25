#include "mosfet.h"

void mosfet_init(mosfet_t *mosfet)
{
    gpio_reset_pin(mosfet->mosfet_pin);
    gpio_set_direction(mosfet->mosfet_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(mosfet->mosfet_pin, 0); // Buzzer off
}

void mosfet_on(mosfet_t *mosfet)
{
    gpio_set_level(mosfet->mosfet_pin, 1); // Buzzer on
}
void mosfet_off(mosfet_t *mosfet)
{
    gpio_set_level(mosfet->mosfet_pin, 0); // Buzzer off
}

void mosfet_blink(mosfet_t *mosfet, int high_interval_ms, int low_interval_ms)
{
    while (1)
    {
        mosfet_on(mosfet);
        vTaskDelay(pdMS_TO_TICKS(high_interval_ms));
        mosfet_off(mosfet);
        vTaskDelay(pdMS_TO_TICKS(low_interval_ms));
    }
}