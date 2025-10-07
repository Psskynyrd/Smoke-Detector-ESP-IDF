#include "ble_handler.h"

#define GATTS_TAG "BLE_HANDLER"

ble_t ble_data;

// Use proper 128-bit UUIDs
static uint8_t service_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};

static uint8_t char_write_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};

static uint8_t char_notify_uuid[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};

#define GATTS_NUM_HANDLE_TEST 6

enum
{
    IDX_SVC,
    IDX_CHAR_WRITE,
    IDX_CHAR_WRITE_VAL,
    IDX_CHAR_NOTIFY,
    IDX_CHAR_NOTIFY_VAL,
    IDX_CHAR_NOTIFY_CFG, 
    HRS_IDX_NB,
};

static uint16_t handle_table[HRS_IDX_NB];
static esp_gatt_if_t gatts_if_global;
static uint16_t conn_id_global = 0xffff;

// Characteristic properties
static uint8_t char_write_prop = ESP_GATT_CHAR_PROP_BIT_WRITE;
static uint8_t char_notify_prop = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

// CCC descriptor value (for notifications)
static uint8_t char_notify_ccc[2] = {0x00, 0x00};

// Characteristic values
static uint8_t char_write_value[512] = {0};
static uint8_t char_notify_value[512] = "BLE_READY";

// Standard GATT UUIDs
static uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

// Advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);

// Fixed GATT database - corrected service UUID reference
static esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] = {
    // Service Declaration - FIXED: proper reference to service UUID
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(service_uuid), sizeof(service_uuid), (uint8_t *)service_uuid}},

    // Write Characteristic Declaration
    [IDX_CHAR_WRITE] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(char_write_prop), &char_write_prop}},

    // Write Characteristic Value
    [IDX_CHAR_WRITE_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)char_write_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(char_write_value), sizeof(char_write_value), (uint8_t *)char_write_value}},

    // Notify Characteristic Declaration
    [IDX_CHAR_NOTIFY] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(char_notify_prop), &char_notify_prop}},

    // Notify Characteristic Value
    [IDX_CHAR_NOTIFY_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)char_notify_uuid, ESP_GATT_PERM_READ, sizeof(char_notify_value), sizeof(char_notify_value), (uint8_t *)char_notify_value}},

    // Client Characteristic Configuration Descriptor
    [IDX_CHAR_NOTIFY_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(char_notify_ccc), (uint8_t *)char_notify_ccc}},
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t ret;

    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Advertising data set complete");
        ret = esp_ble_gap_start_advertising(&adv_params);
        if (ret != ESP_OK)
        {
            ESP_LOGE(GATTS_TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
        }
        break;

    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Raw advertising data set complete");
        ret = esp_ble_gap_start_advertising(&adv_params);
        if (ret != ESP_OK)
        {
            ESP_LOGE(GATTS_TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Advertising started successfully");
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed");
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully");
        }
        break;

    default:
        ESP_LOGI(GATTS_TAG, "GAP event: %d", event);
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep)
        {
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d",
                     param->write.handle, param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);

            // Handle write to characteristic value
            if (handle_table[IDX_CHAR_WRITE_VAL] == param->write.handle)
            {
                // Store received data in the global buffer
                if (param->write.len < sizeof(ble_data.received_message))
                {
                    memset(ble_data.received_message, 0, sizeof(ble_data.received_message));
                    memcpy(ble_data.received_message, param->write.value, param->write.len);
                    ble_data.received_message[param->write.len] = '\0'; // Null terminate
                    ble_data.received_message_len = param->write.len;
                    ble_data.new_message_available = true;

                    ESP_LOGI(GATTS_TAG, "Received message: %s (len=%d)", ble_data.received_message, ble_data.received_message_len);

                    // Call callback if registered
                    if (ble_data.message_received_callback != NULL)
                    {
                        ble_data.message_received_callback(ble_data.received_message, ble_data.received_message_len);
                    }
                }
                else
                {
                    ESP_LOGE(GATTS_TAG, "Received message too long: %d bytes", param->write.len);
                }

                // Also store in char_write_value for compatibility
                if (param->write.len < sizeof(char_write_value))
                {
                    memset(char_write_value, 0, sizeof(char_write_value));
                    memcpy(char_write_value, param->write.value, param->write.len);
                }
            }

            // Handle write to CCC descriptor (notification enable/disable)
            if (handle_table[IDX_CHAR_NOTIFY_CFG] == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001)
                {
                    ESP_LOGI(GATTS_TAG, "Notify enabled");
                    conn_id_global = param->write.conn_id;
                    // Send immediate notification
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, handle_table[IDX_CHAR_NOTIFY_VAL],
                                                strlen((char *)char_notify_value), char_notify_value, false);
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(GATTS_TAG, "Notify disabled");
                }
                // Update the CCC value
                memcpy(char_notify_ccc, param->write.value, 2);
            }
        }

        // Send response for write requests
        if (param->write.need_rsp)
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        }
        break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, handle = %d", param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;

        if (handle_table[IDX_CHAR_NOTIFY_VAL] == param->read.handle)
        {
            rsp.attr_value.len = strlen((char *)char_notify_value);
            memcpy(rsp.attr_value.value, char_notify_value, rsp.attr_value.len);
        }
        else if (handle_table[IDX_CHAR_WRITE_VAL] == param->read.handle)
        {
            rsp.attr_value.len = strlen((char *)char_write_value);
            memcpy(rsp.attr_value.value, char_write_value, rsp.attr_value.len);
        }

        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;

    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TAG, "GATTS event: %d", event);

    // Store global interface
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gatts_if_global = gatts_if;
        }
        else
        {
            ESP_LOGE(GATTS_TAG, "Register app failed, app_id %04x, status %d",
                     param->reg.app_id, param->reg.status);
            return;
        }
    }

    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);

        // Set device name before creating attribute table
        esp_err_t ret = esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
        if (ret != ESP_OK)
        {
            ESP_LOGE(GATTS_TAG, "Set device name failed: %s", esp_err_to_name(ret));
        }

        // Create attribute table
        ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, 0);
        if (ret != ESP_OK)
        {
            ESP_LOGE(GATTS_TAG, "Create attr table failed: %s", esp_err_to_name(ret));
        }
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != HRS_IDX_NB)
        {
            ESP_LOGE(GATTS_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)",
                     param->add_attr_tab.num_handle, HRS_IDX_NB);
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Create attribute table successfully, the number handle = %d", param->add_attr_tab.num_handle);
            memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));

            // Start the service
            esp_err_t ret = esp_ble_gatts_start_service(handle_table[IDX_SVC]);
            if (ret != ESP_OK)
            {
                ESP_LOGE(GATTS_TAG, "Start service failed: %s", esp_err_to_name(ret));
            }
        }
        break;
    }

    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        if (param->start.status == ESP_GATT_OK)
        {
            // Service started successfully, now configure advertising
            ESP_LOGI(GATTS_TAG, "Service started, configuring advertising...");
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        conn_id_global = param->connect.conn_id;

        // Update connection parameters
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = %d", param->disconnect.reason);
        conn_id_global = 0xffff;
        // Reset notification state
        memset(char_notify_ccc, 0, sizeof(char_notify_ccc));
        // Restart advertising
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d",
                 param->conf.status, param->conf.handle);
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;

    default:
        gatts_profile_event_handler(event, gatts_if, param);
        break;
    }
}

// Task to send periodic notifications (optional)
static void notification_task(void *pvParameters)
{
    char notify_data[20];
    uint32_t counter = 0;

    while (1)
    {
        if (conn_id_global != 0xffff && char_notify_ccc[0] == 0x01)
        {
            snprintf(notify_data, sizeof(notify_data), "Count: %lu", counter++);
            esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global, handle_table[IDX_CHAR_NOTIFY_VAL],
                                        strlen(notify_data), (uint8_t *)notify_data, false);
            ESP_LOGI(GATTS_TAG, "Sent notification: %s", notify_data);
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Send every 5 seconds
    }
}

void ble_init(void)
{
    esp_err_t ret;

    ESP_LOGI(GATTS_TAG, "Starting BLE GATT Server...");

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(GATTS_TAG, "NVS initialized");

    // Release memory for classic BT (not needed for BLE)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    ESP_LOGI(GATTS_TAG, "Classic BT memory released");

    // Initialize and enable BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(GATTS_TAG, "BT controller initialized and enabled");

    // Initialize and enable Bluedroid
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(GATTS_TAG, "Bluedroid initialized and enabled");

    // Register GATT and GAP callbacks
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ESP_LOGI(GATTS_TAG, "GATT and GAP callbacks registered");

    // Register GATT application
    ret = esp_ble_gatts_app_register(0);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    // Set TX power to maximum for better range
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    ESP_LOGI(GATTS_TAG, "TX power set to maximum");

    // Wait a bit for everything to initialize
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Configure simple advertising data for maximum compatibility
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x0006, // 7.5ms
        .max_interval = 0x0010, // 20ms
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 16,
        .p_service_uuid = service_uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "Config advertising data failed: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(GATTS_TAG, "Advertising data configured");
    }

    // Optional: Create task for periodic notifications
    // xTaskCreate(notification_task, "notification_task", 4096, NULL, 5, NULL);

    ESP_LOGI(GATTS_TAG, "BLE GATT Server initialization complete");
    ESP_LOGI(GATTS_TAG, "Device should now be discoverable as 'ESP32_BLE_Server'");
}

/**
 * Send long text message via BLE notification (handles messages up to 800 bytes)
 * Automatically splits into multiple packets if needed
 * 
 * @param message: Message string to send (supports up to 800 bytes)
 * @return ESP_OK if message sent successfully, ESP_FAIL otherwise
 */
esp_err_t send_message(const char* message)
{
    // Check if client is connected and notifications are enabled
    if (conn_id_global == 0xffff || char_notify_ccc[0] != 0x01)
    {
        ESP_LOGW(GATTS_TAG, "No client connected or notifications not enabled");
        return ESP_FAIL;
    }

    if (message == NULL)
    {
        ESP_LOGE(GATTS_TAG, "Message is NULL");
        return ESP_FAIL;
    }

    int message_len = strlen(message);
    
    if (message_len == 0)
    {
        ESP_LOGW(GATTS_TAG, "Message is empty");
        return ESP_FAIL;
    }

    ESP_LOGI(GATTS_TAG, "Sending message: %d bytes", message_len);

    // Determine MTU size (default is 23 bytes, but can be negotiated up to 512)
    // Safe chunk size: MTU - 3 (for BLE overhead)
    // Using conservative 100 bytes per chunk for compatibility
    const int CHUNK_SIZE = 100;
    
    // If message fits in one packet, send directly
    if (message_len <= CHUNK_SIZE)
    {
        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_global,
                                                    conn_id_global,
                                                    handle_table[IDX_CHAR_NOTIFY_VAL],
                                                    message_len,
                                                    (uint8_t *)message,
                                                    false);

        if (ret == ESP_OK)
        {
            ESP_LOGI(GATTS_TAG, "✅ Message sent: %s", message);
        }
        else
        {
            ESP_LOGE(GATTS_TAG, "❌ Failed to send message: %s", esp_err_to_name(ret));
        }

        return ret;
    }

    // Message is too long - split into multiple packets
    ESP_LOGI(GATTS_TAG, "Message too long (%d bytes), splitting into chunks...", message_len);
    
    int total_chunks = (message_len + CHUNK_SIZE - 1) / CHUNK_SIZE;
    int offset = 0;
    
    for (int chunk = 0; chunk < total_chunks; chunk++)
    {
        int remaining = message_len - offset;
        int chunk_len = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        
        ESP_LOGI(GATTS_TAG, "Sending chunk %d/%d (%d bytes)", chunk + 1, total_chunks, chunk_len);
        
        // Send chunk
        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_global,
                                                    conn_id_global,
                                                    handle_table[IDX_CHAR_NOTIFY_VAL],
                                                    chunk_len,
                                                    (uint8_t *)(message + offset),
                                                    false);

        if (ret != ESP_OK)
        {
            ESP_LOGE(GATTS_TAG, "❌ Failed to send chunk %d: %s", chunk + 1, esp_err_to_name(ret));
            return ESP_FAIL;
        }

        ESP_LOGI(GATTS_TAG, "✅ Chunk %d/%d sent", chunk + 1, total_chunks);
        
        offset += chunk_len;
        
        // Small delay between chunks to prevent buffer overflow
        // Adjust this delay based on your needs (20-100ms recommended)
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(GATTS_TAG, "✅ Complete message sent successfully (%d chunks)", total_chunks);
    return ESP_OK;
}

/**
 * Alternative: Send message with custom chunk size
 * Use this if you need more control over packet size
 */
esp_err_t send_message_with_chunk_size(const char* message, int chunk_size)
{
    if (conn_id_global == 0xffff || char_notify_ccc[0] != 0x01)
    {
        ESP_LOGW(GATTS_TAG, "No client connected or notifications not enabled");
        return ESP_FAIL;
    }

    if (message == NULL)
    {
        ESP_LOGE(GATTS_TAG, "Message is NULL");
        return ESP_FAIL;
    }

    if (chunk_size <= 0 || chunk_size > 512)
    {
        ESP_LOGE(GATTS_TAG, "Invalid chunk size: %d (valid range: 1-512)", chunk_size);
        return ESP_FAIL;
    }

    int message_len = strlen(message);
    
    if (message_len == 0)
    {
        ESP_LOGW(GATTS_TAG, "Message is empty");
        return ESP_FAIL;
    }

    ESP_LOGI(GATTS_TAG, "Sending %d bytes with chunk size %d", message_len, chunk_size);

    int total_chunks = (message_len + chunk_size - 1) / chunk_size;
    int offset = 0;
    
    for (int chunk = 0; chunk < total_chunks; chunk++)
    {
        int remaining = message_len - offset;
        int current_chunk_len = (remaining > chunk_size) ? chunk_size : remaining;
        
        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_global,
                                                    conn_id_global,
                                                    handle_table[IDX_CHAR_NOTIFY_VAL],
                                                    current_chunk_len,
                                                    (uint8_t *)(message + offset),
                                                    false);

        if (ret != ESP_OK)
        {
            ESP_LOGE(GATTS_TAG, "Failed to send chunk %d/%d", chunk + 1, total_chunks);
            return ESP_FAIL;
        }
        
        offset += current_chunk_len;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(GATTS_TAG, "Complete message sent: %d chunks", total_chunks);
    return ESP_OK;
}

/**
 * Optimized version: Uses larger chunks for better performance
 * Note: Requires char_notify_value[512] or larger in ble_handler.c
 */
esp_err_t send_message_optimized(const char* message)
{
    if (conn_id_global == 0xffff || char_notify_ccc[0] != 0x01)
    {
        ESP_LOGW(GATTS_TAG, "No client connected or notifications not enabled");
        return ESP_FAIL;
    }

    if (message == NULL)
    {
        ESP_LOGE(GATTS_TAG, "Message is NULL");
        return ESP_FAIL;
    }

    int message_len = strlen(message);
    
    if (message_len == 0)
    {
        ESP_LOGW(GATTS_TAG, "Message is empty");
        return ESP_FAIL;
    }

    // Use 244 bytes (MTU 247 - 3 bytes overhead)
    // This works with most modern BLE implementations
    const int CHUNK_SIZE = 244;
    
    if (message_len <= CHUNK_SIZE)
    {
        // Send in one go
        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_global,
                                                    conn_id_global,
                                                    handle_table[IDX_CHAR_NOTIFY_VAL],
                                                    message_len,
                                                    (uint8_t *)message,
                                                    false);
        
        if (ret == ESP_OK)
        {
            ESP_LOGI(GATTS_TAG, "Message sent: %d bytes", message_len);
        }
        
        return ret;
    }

    // Split and send
    int offset = 0;
    int chunk_num = 0;
    
    while (offset < message_len)
    {
        int remaining = message_len - offset;
        int chunk_len = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        
        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_global,
                                                    conn_id_global,
                                                    handle_table[IDX_CHAR_NOTIFY_VAL],
                                                    chunk_len,
                                                    (uint8_t *)(message + offset),
                                                    false);

        if (ret != ESP_OK)
        {
            ESP_LOGE(GATTS_TAG, "Failed sending chunk %d", chunk_num);
            return ESP_FAIL;
        }
        
        chunk_num++;
        offset += chunk_len;
        
        // Shorter delay for better performance
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    ESP_LOGI(GATTS_TAG, "Sent %d bytes in %d chunks", message_len, chunk_num);
    return ESP_OK;
}

/**
 * Send sensor data formatted as JSON
 * This is a convenience function for common sensor data
 */
// esp_err_t send_sensor_data(uint16_t mq2_value, uint8_t fire_detected, 
//                            float temperature, float humidity)
// {
//     char json_message[256];
//     int len = snprintf(json_message, sizeof(json_message),
//                       "{\"mq2\":%d,\"fire\":%d,\"temp\":%.1f,\"hum\":%.1f,\"timestamp\":%lld}",
//                       mq2_value, fire_detected, temperature, humidity,
//                       (long long)(esp_timer_get_time() / 1000));
    
//     if (len >= sizeof(json_message))
//     {
//         ESP_LOGE(GATTS_TAG, "Sensor JSON too long");
//         return ESP_FAIL;
//     }
    
//     return send_message(json_message);
// }

/**
 * Send formatted message (printf-style)
 * Example: send_formatted_message("Temp: %.1f, Humidity: %.1f", temp, hum);
 */
esp_err_t send_formatted_message(const char* format, ...)
{
    char buffer[512];
    va_list args;
    
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len >= sizeof(buffer))
    {
        ESP_LOGE(GATTS_TAG, "Formatted message too long: %d bytes", len);
        return ESP_FAIL;
    }
    
    return send_message(buffer);
}
/**
 * Alternative function to send custom message
 * @param message: Custom message string to send
 * @return ESP_OK if message sent successfully, ESP_FAIL otherwise
 */
esp_err_t send_custom_message(const char *message)
{
    // Check if client is connected and notifications are enabled
    if (conn_id_global == 0xffff || char_notify_ccc[0] != 0x01)
    {
        ESP_LOGW(GATTS_TAG, "No client connected or notifications not enabled");
        return ESP_FAIL;
    }

    if (message == NULL)
    {
        ESP_LOGE(GATTS_TAG, "Message is NULL");
        return ESP_FAIL;
    }

    int len = strlen(message);

    // Check message length (BLE characteristic value limit)
    if (len > 240)
    { // Leave some margin for BLE MTU
        ESP_LOGE(GATTS_TAG, "Message too long: %d bytes", len);
        return ESP_FAIL;
    }

    // Send the notification
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_global,
                                                conn_id_global,
                                                handle_table[IDX_CHAR_NOTIFY_VAL],
                                                len,
                                                (uint8_t *)message,
                                                false);

    if (ret == ESP_OK)
    {
        ESP_LOGI(GATTS_TAG, "Custom message sent: %s", message);
    }
    else
    {
        ESP_LOGE(GATTS_TAG, "Failed to send custom message: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * Check if BLE client is connected and ready to receive notifications
 * @return true if ready to send, false otherwise
 */
bool is_ble_ready_to_send(void)
{
    return (conn_id_global != 0xffff && char_notify_ccc[0] == 0x01);
}

/**
 * Get connection status
 * @return true if client is connected, false otherwise
 */
bool is_ble_connected(void)
{
    return (conn_id_global != 0xffff);
}

// Optional: Callback function pointer for when new message is received
// static void (*message_received_callback)(const char *message, uint16_t len) = NULL;

/**
 * Get the received message
 * @param buffer: Buffer to copy the message into
 * @param buffer_size: Size of the buffer
 * @return Length of the message, or 0 if no message
 */
uint16_t get_received_message(char *buffer, uint16_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0)
    {
        ESP_LOGE(GATTS_TAG, "Invalid buffer");
        return 0;
    }

    if (ble_data.received_message_len == 0)
    {
        ESP_LOGW(GATTS_TAG, "No message available");
        return 0;
    }

    // Copy message to user buffer
    uint16_t copy_len = (ble_data.received_message_len < buffer_size) ? ble_data.received_message_len : (buffer_size - 1);
    memcpy(buffer, ble_data.received_message, copy_len);
    buffer[copy_len] = '\0'; // Null terminate

    ESP_LOGI(GATTS_TAG, "Retrieved message: %s (len=%d)", buffer, copy_len);

    return copy_len;
}

/**
 * Get direct pointer to received message (use with caution)
 * @return Pointer to received message buffer
 */
const char *get_received_message_ptr(void)
{
    return ble_data.received_message;
}

/**
 * Get the length of the received message
 * @return Length of message in bytes
 */
uint16_t get_received_message_length(void)
{
    return ble_data.received_message_len;
}

/**
 * Check if a new message is available
 * @return true if new message available, false otherwise
 */
bool is_new_message_available(void)
{
    return ble_data.new_message_available;
}

/**
 * Clear the new message flag
 */
void clear_new_message_flag(void)
{
    ble_data.new_message_available = false;
}

/**
 * Clear the received message buffer
 */
void clear_received_message(void)
{
    memset(ble_data.received_message, 0, sizeof(ble_data.received_message));
    ble_data.received_message_len = 0;
    ble_data.new_message_available = false;
    ESP_LOGI(GATTS_TAG, "Received message buffer cleared");
}

/**
 * Register a callback function to be called when a new message is received
 * @param callback: Function pointer to callback (NULL to unregister)
 */
void register_message_callback(void (*callback)(const char *message, uint16_t len))
{
    ble_data.message_received_callback = callback;
    if (callback != NULL)
    {
        ESP_LOGI(GATTS_TAG, "Message callback registered");
    }
    else
    {
        ESP_LOGI(GATTS_TAG, "Message callback unregistered");
    }
}