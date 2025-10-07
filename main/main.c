#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp32-dht11.h"
#include "mqtt_client.h"

#include "led.h"

#include "flame-detector.h"
#include "mq2.h"
#include "ble_handler.h"
#include "buzzer.h"

#define LED_PIN GPIO_NUM_23
#define FLAME_SENSOR_PIN GPIO_NUM_27
#define DHT11_PIN GPIO_NUM_4
// #define MQ2_PIN GPIO_NUM_34
#define BUZZER_PIN GPIO_NUM_19
#define MQ2_ADC_CHANNEL ADC1_CHANNEL_6

#define CONFIG_CONNECTION_TIMEOUT 5

static const char *TAG = "Fire Detection System";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
void wifi_init_sta(void);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_publish_example();

led_t led;
flame_detector_t flame_detector;
dht11_t dht11_sensor;
mq2_t mq2_sensor;
buzzer_t buzzer;

void init_sensors()
{
    // LED initialization
    led.led_pin = LED_PIN;
    led_init(&led);

    // Flame detector initialization
    flame_detector.flame_sensor_pin = FLAME_SENSOR_PIN;
    flame_detector_init(&flame_detector);

    // DHT11 initialization
    dht11_sensor.dht11_pin = DHT11_PIN;

    // Buzzer initialization
    buzzer.buzzer_pin = BUZZER_PIN;
    buzzer_init(&buzzer);

    // MQ2 initialization
    mq2_sensor.mq2_adc_channel = MQ2_ADC_CHANNEL;
    mq2_init(&mq2_sensor);
}

void sensor_task(void *pvParameters)
{
    uint8_t fire_detected = 0;
    uint16_t mq2_raw = 0;
    float mq2_value = 0, temperature = 0, humidity = 0;

    ESP_LOGI(TAG, "Starting sensor task");

    while (1)
    {
        // Flame detector
        if (flame_detector_read(&flame_detector))
        {
            if (flame_detector.is_flame)
            {
                led_open(&led);
                ESP_LOGW(TAG, "🔥 Flame detected!");
                // buzzer_blink(&buzzer, 1000, 1000);
                fire_detected = 1;

                // Activate buzzer for 1 second
                buzzer_on(&buzzer);
                vTaskDelay(pdMS_TO_TICKS(300));
                buzzer_off(&buzzer);
                vTaskDelay(pdMS_TO_TICKS(500));
                buzzer_on(&buzzer);
                vTaskDelay(pdMS_TO_TICKS(300));
                buzzer_off(&buzzer);
            }
            else
            {
                led_off(&led);
                buzzer_off(&buzzer);
                fire_detected = 0;
                ESP_LOGI(TAG, "No flame detected.");
            }
        }

        // DHT11 detection
        if (!dht11_read(&dht11_sensor, CONFIG_CONNECTION_TIMEOUT))
        {
            ESP_LOGI(TAG, "[Temperature]> %.2f", dht11_sensor.temperature);
            ESP_LOGI(TAG, "[Humidity]> %.2f", dht11_sensor.humidity);
            temperature = dht11_sensor.temperature;
            humidity = dht11_sensor.humidity;
        }

        // MQ2 detection
        if (mq2_read(&mq2_sensor))
        {
            mq2_raw = mq2_sensor.raw;
            mq2_value = mq2_sensor.voltage;
            if (mq2_sensor.voltage > 1.5)
            {
                led_open(&led);
                vTaskDelay(pdMS_TO_TICKS(300));
                led_off(&led);
                vTaskDelay(pdMS_TO_TICKS(300));
                led_open(&led);
                vTaskDelay(pdMS_TO_TICKS(300));
                led_off(&led);
                
                ESP_LOGW(TAG, "⚠️ Possible gas/smoke detected! (voltage: %.2f, raw: %d)", mq2_sensor.voltage, mq2_sensor.raw);
            }
            else
            {
                ESP_LOGI(TAG, "No gas/smoke detected. (voltage: %.2f, raw: %d)", mq2_sensor.voltage, mq2_sensor.raw);
            }
        }

        // // Log sensor values
        // ESP_LOGI(TAG, "Sensors - MQ2: %d, Fire: %d, Temp: %.1f°C, Humidity: %.1f%%",
        //          mq2_value, fire_detected, temperature, humidity);

        // Check if BLE is ready to send data
        if (is_ble_ready_to_send())
        {
            // Send sensor data via BLE
            char sensor_data[800];
            snprintf(sensor_data, sizeof(sensor_data),
                     "S:%d|%d|%.2f|%.1f|%.1f",
                     fire_detected, mq2_raw, mq2_value, temperature, humidity);

            esp_err_t ret = send_message(sensor_data);
            // esp_err_t ret = send_message_optimized(sensor_data);
            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG, "Sensor data sent successfully");
            }
        }
        else if (is_ble_connected())
        {
            ESP_LOGW(TAG, "BLE connected but notifications not enabled");
        }
        else
        {
            ESP_LOGW(TAG, "No BLE client connected");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void message_polling_task(void *pvParameters)
{
    char received_msg[240];

    ESP_LOGI(TAG, "Message polling task started");

    while (1)
    {
        // Check if new message is available
        if (is_new_message_available())
        {
            // Get the message
            uint16_t len = get_received_message(received_msg, sizeof(received_msg));

            if (len > 0)
            {
                ESP_LOGI(TAG, "📥 [POLLING] Received: %s (len=%d)", received_msg, len);

                // // Process the message
                // if (strcmp(received_msg, "TEMP") == 0)
                // {
                //     ESP_LOGI(TAG, "🌡️ Temperature request");
                //     // Send temperature data
                //     // send_message(0, 0, 25.5, 60.0);
                // }

                // Clear the flag after processing
                clear_new_message_flag();
            }
        }

        // Check every 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    init_sensors();

    vTaskDelay(pdMS_TO_TICKS(2000));

    ble_init();

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Create sensor reading task
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);

    vTaskDelay(pdMS_TO_TICKS(2000));

    xTaskCreate(message_polling_task, "msg_poll", 4096, NULL, 5, NULL);
    // WiFi initialization
    // ESP_ERROR_CHECK(nvs_flash_init());
    // wifi_init_sta();
}
