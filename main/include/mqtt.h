#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include "mqtt_client.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MQTT event types for callbacks
 */
typedef enum {
    MQTT_HANDLER_EVENT_CONNECTED,
    MQTT_HANDLER_EVENT_DISCONNECTED,
    MQTT_HANDLER_EVENT_SUBSCRIBED,
    MQTT_HANDLER_EVENT_UNSUBSCRIBED,
    MQTT_HANDLER_EVENT_PUBLISHED,
    MQTT_HANDLER_EVENT_DATA,
    MQTT_HANDLER_EVENT_ERROR
} mqtt_handler_event_t;

/**
 * @brief MQTT configuration structure
 */
typedef struct {
    const char *broker_url;          // MQTT broker URL (mqtt:// or mqtts://)
    const char *username;            // Username (NULL if not required)
    const char *password;            // Password (NULL if not required)
    const char *client_id;           // Client ID (NULL for auto-generated)
    uint16_t port;                   // Broker port (0 for default: 1883/8883)
    int keepalive;                   // Keep alive interval in seconds
    bool clean_session;              // Clean session flag
    const char *lwt_topic;           // Last Will Topic (NULL if not used)
    const char *lwt_msg;             // Last Will Message
    int lwt_qos;                     // Last Will QoS
    int lwt_retain;                  // Last Will Retain flag
    const char *cert_pem;            // CA certificate for TLS (NULL if not used)
} mqtt_handler_config_t;

/**
 * @brief MQTT data structure for received messages
 */
typedef struct {
    char *topic;
    int topic_len;
    char *data;
    int data_len;
    int qos;
    bool retain;
} mqtt_handler_data_t;

/**
 * @brief Callback function type for MQTT events
 * 
 * @param event Event type
 * @param data Event data (only valid for MQTT_EVENT_DATA)
 * @param user_ctx User context passed during initialization
 */
typedef void (*mqtt_handler_event_cb_t)(mqtt_handler_event_t event, 
                                        mqtt_handler_data_t *data, 
                                        void *user_ctx);

/**
 * @brief Initialize MQTT handler
 * 
 * @param config MQTT configuration
 * @param event_callback Event callback function
 * @param user_ctx User context to pass to callback
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mqtt_handler_init(const mqtt_handler_config_t *config,
                            mqtt_handler_event_cb_t event_callback,
                            void *user_ctx);

/**
 * @brief Start MQTT client
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mqtt_handler_start(void);

/**
 * @brief Stop MQTT client
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mqtt_handler_stop(void);

/**
 * @brief Destroy MQTT handler and free resources
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mqtt_handler_destroy(void);

/**
 * @brief Subscribe to a topic
 * 
 * @param topic Topic to subscribe to
 * @param qos QoS level (0, 1, or 2)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mqtt_handler_subscribe(const char *topic, int qos);

/**
 * @brief Unsubscribe from a topic
 * 
 * @param topic Topic to unsubscribe from
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mqtt_handler_unsubscribe(const char *topic);

/**
 * @brief Publish a message
 * 
 * @param topic Topic to publish to
 * @param data Message data
 * @param len Data length (0 for null-terminated string)
 * @param qos QoS level (0, 1, or 2)
 * @param retain Retain flag
 * @return int Message ID on success, -1 on failure
 */
int mqtt_handler_publish(const char *topic, const char *data, int len, 
                        int qos, int retain);

/**
 * @brief Check if MQTT client is connected
 * 
 * @return true if connected, false otherwise
 */
bool mqtt_handler_is_connected(void);

/**
 * @brief Get the underlying MQTT client handle
 * 
 * @return esp_mqtt_client_handle_t Client handle
 */
esp_mqtt_client_handle_t mqtt_handler_get_client(void);

#ifdef __cplusplus
}
#endif

#endif // MQTT_HANDLER_H