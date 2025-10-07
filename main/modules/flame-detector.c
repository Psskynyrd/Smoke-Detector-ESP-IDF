#include "flame-detector.h"

void flame_detector_init(flame_detector_t *flame_detector)
{
    gpio_reset_pin(flame_detector->flame_sensor_pin);
    gpio_set_direction(flame_detector->flame_sensor_pin, GPIO_MODE_INPUT);
    gpio_pullup_en(flame_detector->flame_sensor_pin);

}

int flame_detector_read(flame_detector_t *flame_detector)
{
    int state = gpio_get_level(flame_detector->flame_sensor_pin);
    if (state == 0)
        flame_detector->is_flame = true;
    else
        flame_detector->is_flame = false;
    return 1;
}
