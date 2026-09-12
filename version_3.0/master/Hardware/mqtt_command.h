#ifndef MQTT_COMMAND_H
#define MQTT_COMMAND_H

#include <stdint.h>
#include "app_protocol.h"

typedef struct {
    uint8_t node;
    uint8_t code;
    uint32_t id;
    uint32_t value;
} MqttCommand;

typedef enum {
    MQTT_COMMAND_OK = 0,
    MQTT_COMMAND_BAD_FORMAT,
    MQTT_COMMAND_UNKNOWN_NODE,
    MQTT_COMMAND_BAD_TOPIC_NODE,
    MQTT_COMMAND_BAD_RELAY
} MqttCommandResult;

/* 解析command/set JSON，并把channel/state转换成无线协议value。 */
MqttCommandResult mqtt_command_parse(const char *topic, const char *json, const uint8_t *known_nodes,  uint8_t node_count,  MqttCommand *command);

/* 解析command/set JSON，并把channel/state转换成无线协议value。 */
int mqtt_relay_state_json(char *out, uint64_t out_size, uint8_t node,
                          uint32_t command_id, uint8_t state_mask);
#endif