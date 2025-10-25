#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp32-dht11.h"
#include <string.h>
#include <esp_mac.h>
#include <stdlib.h>
#include "cJSON.h"

// internet
#include "internet.h"
#include "mqtt.h"

// device
#include "led.h"
#include "flame-detector.h"
#include "mq2.h"
#include "ble_handler.h"
#include "mosfet.h"

#include "config.h"

#define LED_PIN GPIO_NUM_23
#define FLAME_SENSOR_PIN GPIO_NUM_27
#define DHT11_PIN GPIO_NUM_4
// #define MQ2_PIN GPIO_NUM_34
#define MOSFET_PIN GPIO_NUM_19
#define MQ2_ADC_CHANNEL ADC1_CHANNEL_6

#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD
#define WIFI_MAX_RETRY CONFIG_WIFI_MAX_RETRY
#define WIFI_TIMEOUT_MS CONFIG_WIFI_TIMEOUT_MS

#define CONFIG_CONNECTION_TIMEOUT 5

static const char *TAG = "Smoke Detection System";

led_t led;
flame_detector_t flame_detector;
dht11_t dht11_sensor;
mq2_t mq2_sensor;
mosfet_t mosfet;
// ble_t ble_data;

int is_mqtt_connected = 0;
int mosfet_activate = 1;

char device_name_mac[256];
uint8_t mac[6];
uint64_t unique_id = 0;

void get_mac_data()
{
    esp_err_t ret = esp_efuse_mac_get_default(mac);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        for (int i = 0; i < 6; i++)
        {
            unique_id = (unique_id << 8) | mac[i];
        }
        ESP_LOGI(TAG, "Unique ID: %llu", unique_id);

        snprintf(device_name_mac, sizeof(device_name_mac), "%s_%02X%02X%02X", DEVICE_NAME, mac[3], mac[4], mac[5]);
        ESP_LOGI(TAG, "Device Name MAC: %s", device_name_mac);

        snprintf(ble_data.device_name, sizeof(ble_data.device_name), "%s", device_name_mac);
        ESP_LOGW(TAG, "BLE Device Name set to: %s", ble_data.device_name);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to get MAC address (%s)", esp_err_to_name(ret));
    }
}

// #pragma region initialization
void sensor_init()
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
    mosfet.mosfet_pin = MOSFET_PIN;
    mosfet_init(&mosfet);

    // MQ2 initialization
    mq2_sensor.mq2_adc_channel = MQ2_ADC_CHANNEL;
    mq2_init(&mq2_sensor);
}

// MQTT event callback
void mqtt_event_callback(mqtt_handler_event_t event, mqtt_handler_data_t *data, void *user_ctx)
{
    switch (event)
    {
    case MQTT_HANDLER_EVENT_CONNECTED:
        is_mqtt_connected = 1;
        ESP_LOGI(TAG, "MQTT Connected!");

        // Subscribe to topics after connection
        mqtt_handler_subscribe("test/topic", 1);

        // Publish connection status
        mqtt_handler_publish("device/status", "online", 0, 1, 1);
        break;

    case MQTT_HANDLER_EVENT_DISCONNECTED:
        is_mqtt_connected = 0;
        ESP_LOGW(TAG, "MQTT Disconnected");
        break;

    case MQTT_HANDLER_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "Successfully subscribed");
        break;

    case MQTT_HANDLER_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "Message published");
        break;

    case MQTT_HANDLER_EVENT_DATA:
        if (data)
        {
            ESP_LOGI(TAG, "MQTT Message Received:");
            ESP_LOGI(TAG, "  Topic: %.*s", data->topic_len, data->topic);
            ESP_LOGI(TAG, "  Data: %.*s", data->data_len, data->data);
        }
        break;

    case MQTT_HANDLER_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT Error occurred");
        break;

    default:
        break;
    }
}

void mqtt_init()
{
    ESP_LOGI(TAG, "Initializing MQTT...");

    mqtt_handler_config_t config = {
        .broker_url = MQTT_BROKER_HOST,
        .port = MQTT_BROKER_PORT,
        .client_id = MQTT_CLIENT_ID,
        .username = MQTT_USERNAME,
        .password = MQTT_PASSWORD,
        .keepalive = 120,
        .clean_session = true,
        .lwt_topic = "sensor/status",
        .lwt_msg = "offline",
        .lwt_qos = 1,
        .lwt_retain = 1,
        .cert_pem = NULL};

    esp_err_t err = mqtt_handler_init(&config, mqtt_event_callback, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MQTT: %s", esp_err_to_name(err));
        return;
    }

    err = mqtt_handler_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start MQTT: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "MQTT started successfully");
}

// internet init
void internet_init()
{
    ESP_LOGI(TAG, "Wifi Initializing ");
    wifi_init();
    internet_config_t wifi_config = {.ssid = WIFI_SSID, .password = WIFI_PASS, .max_retry = WIFI_MAX_RETRY};
    esp_err_t ret = wifi_connect_with_config(&wifi_config);
}
// #pragma endregion

// #pragma region task
void sensor_task(void *pvParameters)
{
    uint8_t fire_detected = 0;
    float mq2_raw = 0, mq2_value = 0, temperature = 0, humidity = 0;

    ESP_LOGI(TAG, "Starting sensor task");

    while (1)
    {
        if (flame_detector_read(&flame_detector))
        {
            if (flame_detector.is_flame)
            {
                ESP_LOGW(TAG, "🔥 Flame detected!");
                fire_detected = 1;

                if (mosfet_activate)
                {
                    mosfet_activate = 0;
                }
            }
            else
            {
                fire_detected = 0;
                ESP_LOGI(TAG, "No flame detected.");

                if (!mosfet_activate)
                {
                    mosfet_activate = 1;
                }
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
                ESP_LOGW(TAG, "⚠️ Possible gas/smoke detected! (voltage: %.2f, raw: %d)", mq2_sensor.voltage, mq2_sensor.raw);
            }
            else
            {
                ESP_LOGI(TAG, "No gas/smoke detected. (voltage: %.2f, raw: %d)", mq2_sensor.voltage, mq2_sensor.raw);
            }
        }

        // if (is_ble_ready_to_send())
        // {
        //     char alert_msg[400];
        //     snprintf(alert_msg, sizeof(alert_msg), "BLE|CONNECTED| BLE connected!|%s|%llu|%02X:%02X:%02X:%02X:%02X:%02X", device_name_mac, unique_id, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        //     esp_err_t ret = send_custom_message(alert_msg);
        //     if (ret == ESP_OK)
        //     {
        //         ESP_LOGI(TAG, "Alert sent via BLE successfully");
        //     }
        // }
        // else if (is_ble_connected())
        // {
        //     ESP_LOGW(TAG, "BLE connected but notifications not enabled");
        // }
        // else
        // {
        //     ESP_LOGW(TAG, "No BLE client connected");
        // }

        // const char *json_data = "{\"mq2\":%.2f,\"fire\":%d,\"temperature\":%.2f,\"humidity\":%.2f}";
        // snprintf(json_data, sizeof(json_data), json_data, mq2_value, fire_detected, temperature, humidity);

        wifi_status_t current_wifi_status = wifi_get_status();
        ESP_LOGI(TAG, "Current WiFi status: %s", current_wifi_status == WIFI_STATUS_CONNECTED ? "CONNECTED" : current_wifi_status == WIFI_STATUS_CONNECTING ? "CONNECTING"
                                                                                                          : current_wifi_status == WIFI_STATUS_DISCONNECTED ? "DISCONNECTED"
                                                                                                                                                            : "FAILED");

        switch (current_wifi_status)
        {
        case WIFI_STATUS_CONNECTED:
            char json_data[800];
            snprintf(json_data, sizeof(json_data), "{\"uniqueId\":%llu,\"fire\":%d,\"mq2Raw\":%.2f,\"mq2Value\":%.2f,\"temperature\":%.2f,\"humidity\":%.2f}",
                     unique_id, fire_detected, mq2_raw, mq2_value, temperature, humidity);

            // if (is_ble_ready_to_send())
            // {
            //     char alert_msg[] = "WIFI_MSG|CONNECTED| WiFi connected!";
            //     esp_err_t ret = send_custom_message(alert_msg);
            //     if (ret == ESP_OK)
            //     {
            //         ESP_LOGI(TAG, "Alert sent via BLE successfully");
            //     }
            // }
            // else if (is_ble_connected())
            // {
            //     ESP_LOGW(TAG, "BLE connected but notifications not enabled");
            // }
            // else
            // {
            //     ESP_LOGW(TAG, "No BLE client connected");
            // }

            if (!is_mqtt_connected)
            {
                ESP_LOGW(TAG, "MQTT not connected, attempting to reconnect...");
                mqtt_init();
                vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for connection
            }
            else
            {
                ESP_LOGI(TAG, "MQTT is connected.");
                mqtt_handler_publish("sensor/status", json_data, 0, 1, 0);
            }
            break;
        case WIFI_STATUS_CONNECTING:
            ESP_LOGI(TAG, "WiFi is connecting...");
            break;
        case WIFI_STATUS_DISCONNECTED:
            ESP_LOGW(TAG, "WiFi is disconnected, attempting to reconnect...");
            internet_init();
            break;
        case WIFI_STATUS_FAILED:
            ESP_LOGE(TAG, "WiFi connection failed.");
            // if (is_ble_ready_to_send())
            // {
            //     char alert_msg[] = "WIFI_MSG|FAILED| WiFi connection failed!";
            //     esp_err_t ret = send_custom_message(alert_msg);
            //     if (ret == ESP_OK)
            //     {
            //         ESP_LOGI(TAG, "Alert sent via BLE successfully");
            //     }
            // }
            // else if (is_ble_connected())
            // {
            //     ESP_LOGW(TAG, "BLE connected but notifications not enabled");
            // }
            // else
            // {
            //     ESP_LOGW(TAG, "No BLE client connected");
            // }
            break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

static void trim_whitespace(char *str)
{
    char *end;

    // Trim leading space
    while (isspace((unsigned char)*str))
        str++;

    // All spaces?
    if (*str == 0)
        return;

    // Trim trailing space
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end))
        end--;

    // Write new null terminator
    *(end + 1) = '\0';
}

void message_polling_task(void *pvParameters)
{
    char received_msg[800];

    ESP_LOGI(TAG, "Message polling task started");

    while (1)
    {
        if (is_new_message_available())
        {
            uint16_t len = get_received_message(received_msg, sizeof(received_msg));

            if (len > 0)
            {
                ESP_LOGI(TAG, "📥 [POLLING] Received: %s (len=%d)", received_msg, len);

                cJSON *root = cJSON_Parse(received_msg);
                if (!root)
                {
                    printf("Failed to parse JSON\n");
                    return;
                }

                cJSON *cmd_item = cJSON_GetObjectItem(root, "command");
                cJSON *ts_item = cJSON_GetObjectItem(root, "timestamp");
                cJSON *data = cJSON_GetObjectItem(root, "data");

                if (!cmd_item || !ts_item || !data)
                {
                    printf("Invalid JSON structure\n");
                    cJSON_Delete(root);
                    return;
                }

                const char *command = cmd_item->valuestring;
                long timestamp = (long)ts_item->valuedouble;

                cJSON *ssid_item = cJSON_GetObjectItem(data, "ssid");
                cJSON *password_item = cJSON_GetObjectItem(data, "password");

                if (!ssid_item || !password_item)
                {
                    printf("Missing WiFi credentials\n");
                    cJSON_Delete(root);
                    return;
                }

                const char *ssid_str = ssid_item->valuestring;
                const char *password_str = password_item->valuestring;

                char ssid[64];
                char password[64];
                strncpy(ssid, ssid_str, sizeof(ssid) - 1);
                strncpy(password, password_str, sizeof(password) - 1);
                ssid[sizeof(ssid) - 1] = '\0';
                password[sizeof(password) - 1] = '\0';

                char command_buf[64];
                strncpy(command_buf, command, sizeof(command_buf) - 1);
                command_buf[sizeof(command_buf) - 1] = '\0';
                trim_whitespace(command_buf);

                printf("Command: %s\n", command);
                printf("Command (trimmed): '%s'\n", command_buf);
                printf("Timestamp: %ld\n", timestamp);
                printf("SSID: %s\n", ssid);
                printf("Password: %s\n", password);

                cJSON_Delete(root);
                vTaskDelay(pdMS_TO_TICKS(3000));

                // Command handling
                if (strcmp(command_buf, "WIFI_CONNECT") == 0)
                {
                    ESP_LOGI(TAG, "Received WIFI_CONNECT command");
                    wifi_init();
                    internet_config_t wifi_config;

                    strncpy(wifi_config.ssid, ssid, sizeof(wifi_config.ssid) - 1);
                    wifi_config.ssid[sizeof(wifi_config.ssid) - 1] = '\0';

                    strncpy(wifi_config.password, password, sizeof(wifi_config.password) - 1);
                    wifi_config.password[sizeof(wifi_config.password) - 1] = '\0';

                    wifi_config.max_retry = WIFI_MAX_RETRY;

                    wifi_connect_with_config(&wifi_config);
                }
                else if (strcmp(command, "BLE_DISCONNECT") == 0)
                {
                    disconnect_ble();
                }

                clear_new_message_flag();
            }
        }

        // Check every 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
// #pragma endregion

void app_main(void)
{
    get_mac_data();
    vTaskDelay(pdMS_TO_TICKS(2000));

    sensor_init();
    vTaskDelay(pdMS_TO_TICKS(2000));

    internet_init();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // ble_init();
    // vTaskDelay(pdMS_TO_TICKS(2000));

    // xTaskCreate(message_polling_task, "msg_poll", 4096, NULL, 5, NULL);
    // vTaskDelay(pdMS_TO_TICKS(200));

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // while (1)
    // {
    if (mosfet_activate)
    {
        ESP_LOGI(TAG, "Mosfet ON");
        // mosfet_on(&mosfet);
        gpio_set_level(MOSFET_PIN, 1);
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    else
    {
        ESP_LOGI(TAG, "Mosfet OFF");
        // mosfet_off(&mosfet);
        gpio_set_level(MOSFET_PIN, 0);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // }
}
