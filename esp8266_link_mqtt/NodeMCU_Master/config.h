#ifndef GATEWAY_CONFIG_H
#define GATEWAY_CONFIG_H

#define WIFI_SSID             "YOUR_WIFI_SSID"
#define WIFI_PASSWORD         "YOUR_WIFI_PASSWORD"
#define MQTT_HOST             "192.168.1.10"
#define MQTT_PORT             1883
#define MQTT_USER             ""
#define MQTT_PASSWORD         ""
#define GATEWAY_ID            "gateway01"

#define DEBUG_ENABLED         1
#define OLED_ENABLED          1
#define OLED_I2C_ADDRESS      0x3C
#define OLED_SDA_PIN          D3  /* GPIO0，启动时必须保持高 */
#define OLED_SCL_PIN          D4  /* GPIO2，启动时必须保持高 */

/* 增加节点时在这里添加ID；从机app_config.h必须使用相同ID。 */
static const uint8_t CONFIGURED_NODE_IDS[] = {1};
#define NODE_COUNT (sizeof(CONFIGURED_NODE_IDS)/sizeof(CONFIGURED_NODE_IDS[0]))

#define RF_CHANNEL            76
#define NORMAL_POLL_MS        5000UL
#define OFFLINE_PROBE_MS      30000UL
#define DEGRADED_FAILURES     3
#define OFFLINE_FAILURES      5
#define ONLINE_RECOVERY_OK    2
#define MQTT_BUFFER_SIZE      768

#endif
