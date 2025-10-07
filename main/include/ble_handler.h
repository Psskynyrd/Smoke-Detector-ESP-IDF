#ifndef __BLE_HANDLER__
#define __BLE_HANDLER__
#include <driver/gpio.h>
#include <stdio.h>
#include <string.h>
#include <rom/ets_sys.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>
#include "esp_mac.h"
#include "config.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 5 // 5 = ESP_LOG_VERBOSE
#endif

typedef struct
{
    char received_message[240];
    uint16_t received_message_len;
    bool new_message_available;
    void (*message_received_callback)(const char *message, uint16_t len);
} ble_t;

void ble_init(void);
// esp_err_t send_message(char message);
esp_err_t send_custom_message(const char *message);
bool is_ble_ready_to_send(void);
bool is_ble_connected(void);


/**
 * Send message via BLE (handles long text automatically)
 * Supports messages up to 800 bytes, automatically splits into chunks
 * @param message: Message string to send
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t send_message(const char* message);

/**
 * Send message with custom chunk size
 * @param message: Message string to send
 * @param chunk_size: Size of each chunk (1-512 bytes)
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t send_message_with_chunk_size(const char* message, int chunk_size);

/**
 * Optimized send for better performance (uses 244-byte chunks)
 * @param message: Message string to send
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t send_message_optimized(const char* message);

/**
 * Send sensor data as JSON
 * @param mq2_value: MQ-2 gas sensor reading
 * @param fire_detected: Fire sensor status (0 = no fire, 1 = fire detected)
 * @param temperature: DHT11 temperature in Celsius
 * @param humidity: DHT11 humidity in percentage
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t send_sensor_data(uint16_t mq2_value, uint8_t fire_detected, 
                           float temperature, float humidity);

/**
 * Send formatted message (printf-style)
 * Example: send_formatted_message("Value: %d, Status: %s", val, status);
 * @param format: Printf-style format string
 * @param ...: Variable arguments
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t send_formatted_message(const char* format, ...);

// ============================================================================
// RECEIVING FUNCTIONS
// ============================================================================

/**
 * Get the received message
 * @param buffer: Buffer to copy message into
 * @param buffer_size: Size of the buffer
 * @return Length of message copied, 0 if no message
 */
uint16_t get_received_message(char* buffer, uint16_t buffer_size);

/**
 * Get direct pointer to received message (read-only access)
 * @return Pointer to received message buffer
 */
const char* get_received_message_ptr(void);

/**
 * Get length of received message
 * @return Length in bytes
 */
uint16_t get_received_message_length(void);

/**
 * Check if new message is available
 * @return true if new message available
 */
bool is_new_message_available(void);

/**
 * Clear the new message flag
 */
void clear_new_message_flag(void);

/**
 * Clear the received message buffer
 */
void clear_received_message(void);

/**
 * Register callback for when message is received
 * @param callback: Function pointer (NULL to unregister)
 *                  void callback(const char* message, uint16_t len)
 */
void register_message_callback(void (*callback)(const char* message, uint16_t len));


#endif