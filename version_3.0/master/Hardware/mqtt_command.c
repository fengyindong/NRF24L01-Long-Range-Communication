#include "mqtt_command.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* 读取一个无符号JSON整数；仅处理本项目固定的扁平命令对象。 */
static uint8_t json_u32(const char *json, const char *name, uint32_t *value)
{
    char key[32];
    const char *p;
    char *end;
    size_t n = strlen(name);
    if (!json || !value || n + 3u > sizeof(key)) return 0u;
    key[0] = '"'; memcpy(key + 1, name, n); key[n + 1u] = '"'; key[n + 2u] = 0;
    p = strstr(json, key);
    if (!p || !(p = strchr(p + n + 2u, ':'))) return 0u;
    *value = (uint32_t)strtoul(p + 1, &end, 10);
    return end != p + 1;
}

/* 从主题中的/node/<id>/取出目标节点。 */
static uint8_t topic_node_id(const char *topic, uint32_t *node)
{
    const char *p;
    char *end;
    if (!topic || !node || !(p = strstr(topic, "/node/"))) return 0u;
    *node = (uint32_t)strtoul(p + 6, &end, 10);
    return end != p + 6 && strcmp(end, "/command/set") == 0;
}

MqttCommandResult mqtt_command_parse(const char *topic, const char *json,
                                     const uint8_t *known_nodes,
                                     uint8_t node_count,
                                     MqttCommand *command)
{
    uint32_t node, topic_node, id, code, value = 0u, channel, state;
    uint8_t i, known = 0u;
    if (!command || !known_nodes ||
        !json_u32(json, "node_id", &node) ||
        !json_u32(json, "command_id", &id) ||
        !json_u32(json, "code", &code) || !node || !id || !code ||
        !topic_node_id(topic, &topic_node)) return MQTT_COMMAND_BAD_FORMAT;
    if (node != topic_node) return MQTT_COMMAND_BAD_TOPIC_NODE;
    for (i = 0u; i < node_count; ++i) if (known_nodes[i] == node) known = 1u;
    if (!known) return MQTT_COMMAND_UNKNOWN_NODE;

    if (code == APP_CMD_SET_RELAY) {
        if (json_u32(json, "channel", &channel) && json_u32(json, "state", &state)) {
            if (!app_relay_encode((uint8_t)channel, (uint8_t)state, &value))
                return MQTT_COMMAND_BAD_RELAY;
        } else {
            uint8_t ignored;
            if (!json_u32(json, "value", &value) ||
                !app_relay_apply(0u, value, &ignored)) return MQTT_COMMAND_BAD_RELAY;
        }
    } else if (code != APP_CMD_GET_RELAY_STATE) {
        (void)json_u32(json, "value", &value);
    }
    command->node = (uint8_t)node;
    command->id = id;
    command->code = (uint8_t)code;
    command->value = value;
    return MQTT_COMMAND_OK;
}

int mqtt_relay_state_json(char *out, uint64_t out_size, uint8_t node,
                          uint32_t command_id, uint8_t state_mask)
{
    uint8_t state = (uint8_t)(state_mask & 0x0Fu);
    if (!out || !out_size) return -1;
    return snprintf(out, out_size,
        "{\"node_id\":%u,\"command_id\":%lu,\"relay1\":%u,"
        "\"relay2\":%u,\"relay3\":%u,\"relay4\":%u,\"state_mask\":%u}",
        node, (unsigned long)command_id, (state >> 0) & 1u,
        (state >> 1) & 1u, (state >> 2) & 1u, (state >> 3) & 1u, state);
}

