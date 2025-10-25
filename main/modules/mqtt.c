#include "mqtt.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "MQTT";

typedef struct {
    esp_mqtt_client_handle_t client;
    mqtt_handler_event_cb_t event_callback;
    void *user_ctx;
    bool connected;
} mqtt_handler_ctx_t;

static mqtt_handler_ctx_t mqtt_ctx = {0};

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                              int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    mqtt_handler_data_t data = {0};

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            mqtt_ctx.connected = true;
            if (mqtt_ctx.event_callback) {
                mqtt_ctx.event_callback(MQTT_HANDLER_EVENT_CONNECTED, NULL, mqtt_ctx.user_ctx);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_ctx.connected = false;
            if (mqtt_ctx.event_callback) {
                mqtt_ctx.event_callback(MQTT_HANDLER_EVENT_DISCONNECTED, NULL, mqtt_ctx.user_ctx);
            }
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            if (mqtt_ctx.event_callback) {
                mqtt_ctx.event_callback(MQTT_HANDLER_EVENT_SUBSCRIBED, NULL, mqtt_ctx.user_ctx);
            }
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            if (mqtt_ctx.event_callback) {
                mqtt_ctx.event_callback(MQTT_HANDLER_EVENT_UNSUBSCRIBED, NULL, mqtt_ctx.user_ctx);
            }
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            if (mqtt_ctx.event_callback) {
                mqtt_ctx.event_callback(MQTT_HANDLER_EVENT_PUBLISHED, NULL, mqtt_ctx.user_ctx);
            }
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            data.topic = event->topic;
            data.topic_len = event->topic_len;
            data.data = event->data;
            data.data_len = event->data_len;
            data.qos = event->qos;
            data.retain = event->retain;

            if (mqtt_ctx.event_callback) {
                mqtt_ctx.event_callback(MQTT_HANDLER_EVENT_DATA, &data, mqtt_ctx.user_ctx);
            }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Last error code: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "Last tls stack error: 0x%x", event->error_handle->esp_tls_stack_err);
                ESP_LOGE(TAG, "Last captured errno: %d (%s)", event->error_handle->esp_transport_sock_errno,
                         strerror(event->error_handle->esp_transport_sock_errno));
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGE(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            } else {
                ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
            }
            if (mqtt_ctx.event_callback) {
                mqtt_ctx.event_callback(MQTT_HANDLER_EVENT_ERROR, NULL, mqtt_ctx.user_ctx);
            }
            break;

        default:
            ESP_LOGD(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

esp_err_t mqtt_handler_init(const mqtt_handler_config_t *config,
                            mqtt_handler_event_cb_t event_callback,
                            void *user_ctx)
{
    if (!config || !config->broker_url) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    if (mqtt_ctx.client != NULL) {
        ESP_LOGW(TAG, "MQTT already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = config->broker_url,
        .credentials.username = config->username,
        .credentials.authentication.password = config->password,
        .credentials.client_id = config->client_id,
        .session.keepalive = config->keepalive > 0 ? config->keepalive : 60,
        .session.disable_clean_session = !config->clean_session,
    };

    if (config->port > 0) {
        mqtt_cfg.broker.address.port = config->port;
    }

    if (config->lwt_topic) {
        mqtt_cfg.session.last_will.topic = config->lwt_topic;
        mqtt_cfg.session.last_will.msg = config->lwt_msg ? config->lwt_msg : "";
        mqtt_cfg.session.last_will.msg_len = config->lwt_msg ? strlen(config->lwt_msg) : 0;
        mqtt_cfg.session.last_will.qos = config->lwt_qos;
        mqtt_cfg.session.last_will.retain = config->lwt_retain;
    }

    if (config->cert_pem) {
        mqtt_cfg.broker.verification.certificate = config->cert_pem;
    }

    mqtt_ctx.client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_ctx.client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    mqtt_ctx.event_callback = event_callback;
    mqtt_ctx.user_ctx = user_ctx;
    mqtt_ctx.connected = false;

    esp_err_t err = esp_mqtt_client_register_event(mqtt_ctx.client, 
                                                    ESP_EVENT_ANY_ID,
                                                    mqtt_event_handler, 
                                                    NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register event handler: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(mqtt_ctx.client);
        mqtt_ctx.client = NULL;
        return err;
    }

    ESP_LOGI(TAG, "MQTT handler initialized");
    return ESP_OK;
}

esp_err_t mqtt_handler_start(void)
{
    if (mqtt_ctx.client == NULL) {
        ESP_LOGE(TAG, "MQTT not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_mqtt_client_start(mqtt_ctx.client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MQTT client started");
    return ESP_OK;
}

esp_err_t mqtt_handler_stop(void)
{
    if (mqtt_ctx.client == NULL) {
        ESP_LOGE(TAG, "MQTT not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_mqtt_client_stop(mqtt_ctx.client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop MQTT client: %s", esp_err_to_name(err));
        return err;
    }

    mqtt_ctx.connected = false;
    ESP_LOGI(TAG, "MQTT client stopped");
    return ESP_OK;
}

esp_err_t mqtt_handler_destroy(void)
{
    if (mqtt_ctx.client == NULL) {
        ESP_LOGW(TAG, "MQTT not initialized");
        return ESP_OK;
    }

    esp_err_t err = esp_mqtt_client_destroy(mqtt_ctx.client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to destroy MQTT client: %s", esp_err_to_name(err));
        return err;
    }

    mqtt_ctx.client = NULL;
    mqtt_ctx.event_callback = NULL;
    mqtt_ctx.user_ctx = NULL;
    mqtt_ctx.connected = false;

    ESP_LOGI(TAG, "MQTT handler destroyed");
    return ESP_OK;
}

esp_err_t mqtt_handler_subscribe(const char *topic, int qos)
{
    if (mqtt_ctx.client == NULL) {
        ESP_LOGE(TAG, "MQTT not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!mqtt_ctx.connected) {
        ESP_LOGW(TAG, "MQTT not connected, subscription may fail");
    }

    int msg_id = esp_mqtt_client_subscribe(mqtt_ctx.client, topic, qos);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to topic: %s", topic);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Subscribed to topic: %s, msg_id=%d", topic, msg_id);
    return ESP_OK;
}

esp_err_t mqtt_handler_unsubscribe(const char *topic)
{
    if (mqtt_ctx.client == NULL) {
        ESP_LOGE(TAG, "MQTT not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!mqtt_ctx.connected) {
        ESP_LOGW(TAG, "MQTT not connected, unsubscription may fail");
    }

    int msg_id = esp_mqtt_client_unsubscribe(mqtt_ctx.client, topic);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to unsubscribe from topic: %s", topic);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Unsubscribed from topic: %s, msg_id=%d", topic, msg_id);
    return ESP_OK;
}

int mqtt_handler_publish(const char *topic, const char *data, int len,
                        int qos, int retain)
{
    if (mqtt_ctx.client == NULL) {
        ESP_LOGE(TAG, "MQTT not initialized");
        return -1;
    }

    if (!mqtt_ctx.connected) {
        ESP_LOGW(TAG, "MQTT not connected, publish may fail");
    }

    int msg_id = esp_mqtt_client_publish(mqtt_ctx.client, topic, data, len, qos, retain);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish to topic: %s", topic);
        return -1;
    }

    ESP_LOGD(TAG, "Published to topic: %s, msg_id=%d", topic, msg_id);
    return msg_id;
}

bool mqtt_handler_is_connected(void)
{
    return mqtt_ctx.connected;
}

esp_mqtt_client_handle_t mqtt_handler_get_client(void)
{
    return mqtt_ctx.client;
}