# idf_component_register(SRCS "main.c"
#                     INCLUDE_DIRS ".")
# idf_component_register(SRCS "main.c" "dht32-dht11.c" "modules/flame-detector.c" "mq2.c" "led.c"
#                     INCLUDE_DIRS "include"
#                     REQUIRES driver esp_wifi esp_event nvs_flash lwip mqtt)
file(GLOB SRC_FILES
    "*.c"
    "modules/*.c"
)
idf_component_register(SRCS ${SRC_FILES}
    INCLUDE_DIRS "include" "modules" "."
    REQUIRES driver esp_wifi esp_event nvs_flash lwip mqtt bt esp_http_client esp_netif mbedtls json)

# idf_component_register(
#     SRCS ${SRC_FILES}
#     INCLUDE_DIRS "include" "modules" "."
#     REQUIRES driver esp_wifi esp_event nvs_flash lwip mqtt bt esp_http_client esp_websocket esp_netif mbedtls
# )


# Optional: Set component compile options
# target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format)

# Optional: Add preprocessor definitions
# target_compile_definitions(${COMPONENT_LIB} PRIVATE
#     WIFI_SSID="YourSSID"
#     WIFI_PASSWORD="YourPassword"
# )