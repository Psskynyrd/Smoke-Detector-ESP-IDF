#include "mq2.h"

void mq2_init(mq2_t *mq2)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(mq2->mq2_adc_channel, ADC_ATTEN_DB_11);
}

int mq2_read(mq2_t *mq2)
{
    int raw = adc1_get_raw(mq2->mq2_adc_channel);
    float voltage = (raw / 4095.0) * 3.3;

    mq2->raw = raw;
    mq2->voltage = voltage;

    return 1;
}