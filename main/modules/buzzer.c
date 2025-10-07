#include "buzzer.h"

void buzzer_init(buzzer_t *buzzer)
{
    gpio_reset_pin(buzzer->buzzer_pin);
    gpio_set_direction(buzzer->buzzer_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(buzzer->buzzer_pin, 0); // Buzzer off
}

void buzzer_on(buzzer_t *buzzer)
{
    gpio_set_level(buzzer->buzzer_pin, 1); // Buzzer on
}
void buzzer_off(buzzer_t *buzzer)
{
    gpio_set_level(buzzer->buzzer_pin, 0); // Buzzer off
}

void buzzer_blink(buzzer_t *buzzer, int high_interval_ms, int low_interval_ms)
{
    while (1)
    {
        buzzer_on(buzzer);
        vTaskDelay(pdMS_TO_TICKS(high_interval_ms));
        buzzer_off(buzzer);
        vTaskDelay(pdMS_TO_TICKS(low_interval_ms));
    }
}