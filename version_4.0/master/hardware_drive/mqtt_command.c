#include "mqtt_command.h"

#include <stdio.h>
#include <string.h>

/* ============================================================
 * 解析器版本与诊断信息
 * ============================================================ */

/* 最近一次命令解析停留的阶段，便于定位解析失败原因。 */
static const char *last_detail = "not_run";

/**
 * @brief 返回当前 F407 单路继电器命令解析器版本。
 *
 * 用于调试时确认固件是否更新到位。
 */
const char *mqtt_command_parser_version(void)
{
    return "f407-single-relay-v4ascii";
}

/**
 * @brief 返回最近一次命令解析停留的阶段。
 *
 * 可能在以下值之间变化：
 *   "not_run"        —— 尚未运行
 *   "arguments"      —— 参数校验
 *   "json_structure" —— JSON 结构校验
 *   "node_id"        —— 解析 node_id
 *   "command_id"     —— 解析 command_id
 *   "code"           —— 解析 code
 *   "zero_field"     —— 字段为 0 检查
 *   "topic"          —— 解析主题中的节点 ID
 *   "range"          —— 字段范围检查
 *   "topic_node"     —— 主题节点与 JSON 节点一致性检查
 *   "known_node"     —— 节点是否在已知列表中
 *   "relay_fields"   —— 继电器字段解析
 *   "ok"             —— 解析成功
 */
const char *mqtt_command_last_detail(void)
{
    return last_detail;
}

/* ============================================================
 * ASCII 辅助函数
 *
 * 不使用 <ctype.h> 的 isdigit/isspace，避免不同 Keil C 库
 * 实现差异（例如某些精简库对高位字符的处理不一致）。
 * ============================================================ */

/* 仅接受协议需要的 ASCII 数字字符 '0' ~ '9' */
static uint8_t ascii_is_digit(char character)
{
    return (uint8_t)(character >= '0' && character <= '9');
}

/* JSON 允许的空白字符：空格、制表符、回车、换行 */
static uint8_t ascii_is_space(char character)
{
    return (uint8_t)(character == ' ' || character == '\t' ||
                     character == '\r' || character == '\n');
}

/* ============================================================
 * 数字解析
 * ============================================================ */

/**
 * @brief 解析 0 ~ UINT32_MAX 的无符号十进制整数。
 *
 * 不使用 strtoull/errno，避免不同 Keil C 库实现产生不一致行为。
 *
 * @param text  输入字符串
 * @param end   输出参数，返回解析停止位置（数字之后的第一个字符）
 * @param value 输出参数，返回解析结果
 * @return 1 成功；0 失败（无数字、溢出或参数为空）
 */
static uint8_t parse_u32(const char *text, const char **end, uint32_t *value)
{
    uint32_t parsed = 0u;
    uint8_t  digit;

    if (!text || !end || !value) {
        return 0u;
    }

    /* 跳过前导空白 */
    while (ascii_is_space(*text)) {
        ++text;
    }
    if (!ascii_is_digit(*text)) {
        return 0u;
    }

    /* 逐位解析，并检查是否溢出 */
    while (ascii_is_digit(*text)) {
        digit = (uint8_t)(*text - '0');
        if (parsed > (0xFFFFFFFFu - digit) / 10u) {
            return 0u;   /* 溢出 */
        }
        parsed = parsed * 10u + digit;
        ++text;
    }

    *end   = text;
    *value = parsed;
    return 1u;
}

/* ============================================================
 * JSON 结构校验
 * ============================================================ */

/**
 * @brief 校验 JSON 字符串是否为“扁平”的键值对格式，且仅包含已知字段。
 *
 * 支持字段：node_id、command_id、code、channel、state、mask、states、value。
 * 每个字段的值必须为非负十进制整数。
 *
 * 以下情况均判为非法：
 *   - 缺少 '{' 或 '}'，或有多余字符
 *   - 键名重复
 *   - 出现未知字段
 *   - 值的格式不是纯十进制整数
 *   - 值超出 32 位无符号范围
 *
 * @param json 待校验的 JSON 字符串
 * @return 1 合法；0 非法
 */
static uint8_t json_valid(const char *json)
{
    const char *p = json;
    const char *key_start;
    const char *key_end;
    const char *number_end;
    uint32_t    number;
    char        key[32];
    size_t      key_length;
    size_t      count = 0u;
    uint8_t     seen[8] = {0u};

    if (!p) {
        return 0u;
    }

    /* 跳过前导空白，必须以 '{' 开头 */
    while (ascii_is_space(*p)) {
        ++p;
    }
    if (*p++ != '{') {
        return 0u;
    }

    for (;;) {
        static const char *const names[] = {
            "node_id", "command_id", "code", "channel",
            "state",   "mask",       "states", "value"
        };
        size_t index;

        /* 跳过空白，检查是否到达对象结尾 */
        while (ascii_is_space(*p)) {
            ++p;
        }
        if (*p == '}') {
            return count > 0u;   /* 至少要有一个字段 */
        }

        /* 解析键名（必须以 '"' 开头和结尾） */
        if (*p++ != '"') {
            return 0u;
        }
        key_start = p;
        while (*p && *p != '"') {
            ++p;
        }
        if (*p != '"') {
            return 0u;
        }
        key_end    = p++;
        key_length = (size_t)(key_end - key_start);

        /* 键名长度合法且不超过缓冲区 */
        if (!key_length || key_length >= sizeof(key)) {
            return 0u;
        }
        memcpy(key, key_start, key_length);
        key[key_length] = 0;

        /* 跳过空白，检查冒号 */
        while (ascii_is_space(*p)) {
            ++p;
        }
        if (*p++ != ':') {
            return 0u;
        }

        /* 跳过空白，值必须为数字 */
        while (ascii_is_space(*p)) {
            ++p;
        }
        if (!ascii_is_digit(*p)) {
            return 0u;
        }
        if (!parse_u32(p, &number_end, &number)) {
            return 0u;
        }
        p = number_end;

        /* 检查字段名是否在已知列表中，并防止重复 */
        for (index = 0u; index < sizeof(names) / sizeof(names[0]); ++index) {
            if (strcmp(key, names[index]) == 0) {
                if (seen[index]) {
                    return 0u;   /* 字段重复 */
                }
                seen[index] = 1u;
                break;
            }
        }
        ++count;

        /* 跳过空白，判断下一个字符是逗号还是右括号 */
        while (ascii_is_space(*p)) {
            ++p;
        }
        if (*p == ',') {
            ++p;
            while (ascii_is_space(*p)) {
                ++p;
            }
            /* 不允许尾随逗号，如 {"a":1,} */
            if (*p == '}') {
                return 0u;
            }
            continue;
        }
        if (*p == '}') {
            ++p;
            while (ascii_is_space(*p)) {
                ++p;
            }
            return *p == 0;   /* 结尾之后不能再有其他字符 */
        }
        return 0u;
    }
}

/* ============================================================
 * JSON 字段提取
 * ============================================================ */

/**
 * @brief 从 JSON 字符串中提取指定字段的无符号整数值。
 *
 * @param json   JSON 字符串
 * @param name   字段名（不含引号）
 * @param value  输出参数，返回解析结果
 * @return 1 成功；0 失败（字段不存在或值非法）
 */
static uint8_t json_u32(const char *json, const char *name, uint32_t *value)
{
    char        key[32];
    const char *position;
    const char *end;
    size_t      length = strlen(name);
    uint32_t    parsed;

    if (!json || !value || length + 3u > sizeof(key)) {
        return 0u;
    }

    /* 构造带引号的键名，例如 "node_id" */
    key[0] = '"';
    memcpy(key + 1, name, length);
    key[length + 1u] = '"';
    key[length + 2u] = 0;

    /* 查找键名位置，再定位到冒号 */
    position = strstr(json, key);
    if (!position || !(position = strchr(position + length + 2u, ':'))) {
        return 0u;
    }

    /* 解析冒号之后的数字 */
    if (!parse_u32(position + 1, &end, &parsed)) {
        return 0u;
    }

    *value = parsed;
    return 1u;
}

/* ============================================================
 * 主题解析
 * ============================================================ */

/**
 * @brief 从主题中提取节点 ID，并验证主题后缀为 "/command/set"。
 *
 * 主题格式：.../node/<id>/command/set
 *
 * @param topic 主题字符串
 * @param node  输出参数，返回节点 ID
 * @return 1 成功；0 失败
 */
static uint8_t topic_node_id(const char *topic, uint32_t *node)
{
    const char *position;
    const char *end;
    uint32_t    parsed;

    if (!topic || !node || !(position = strstr(topic, "/node/"))) {
        return 0u;
    }

    /* 解析 "/node/" 之后的数字 */
    if (!parse_u32(position + 6, &end, &parsed)) {
        return 0u;
    }

    *node = parsed;

    /* 后缀必须是 "/command/set" */
    return strcmp(end, "/command/set") == 0;
}

/* ============================================================
 * 命令解析主函数
 * ============================================================ */

/**
 * @brief 解析 MQTT 下行命令。
 *
 * 流程：
 *   1) 参数校验；
 *   2) JSON 结构校验；
 *   3) 提取 node_id、command_id、code；
 *   4) 检查字段是否为 0；
 *   5) 校验主题节点 ID；
 *   6) 校验字段范围；
 *   7) 校验主题节点与 JSON 节点是否一致；
 *   8) 校验节点是否在已知列表中；
 *   9) 对继电器类命令做参数编码；
 *  10) 输出解析结果。
 *
 * 每一步都更新 last_detail，便于定位失败位置。
 *
 * @param topic       消息主题
 * @param json        消息载荷（JSON）
 * @param known_nodes 已知节点 ID 列表
 * @param node_count  节点数量
 * @param command     输出参数，解析成功时填入命令内容
 * @return 解析结果码
 */
MqttCommandResult mqtt_command_parse(const char *topic, const char *json,
                                     const uint8_t *known_nodes,
                                     uint8_t node_count,
                                     MqttCommand *command)
{
    uint32_t node;
    uint32_t topic_node;
    uint32_t id;
    uint32_t code;
    uint32_t value = 0u;
    uint32_t channel;
    uint32_t state;
    uint8_t  has_channel;
    uint8_t  has_state;
    uint8_t  index;
    uint8_t  known = 0u;

    /* ---------- 步骤 1：参数校验 ---------- */
    last_detail = "arguments";
    if (!command || !known_nodes || !topic || !json) {
        return MQTT_COMMAND_BAD_FORMAT;
    }

    /* ---------- 步骤 2：JSON 结构校验 ---------- */
    last_detail = "json_structure";
    if (!json_valid(json)) {
        return MQTT_COMMAND_BAD_FORMAT;
    }

    /* ---------- 步骤 3：提取基础字段 ---------- */
    last_detail = "node_id";
    if (!json_u32(json, "node_id", &node)) {
        return MQTT_COMMAND_BAD_FORMAT;
    }

    last_detail = "command_id";
    if (!json_u32(json, "command_id", &id)) {
        return MQTT_COMMAND_BAD_FORMAT;
    }

    last_detail = "code";
    if (!json_u32(json, "code", &code)) {
        return MQTT_COMMAND_BAD_FORMAT;
    }

    /* ---------- 步骤 4：字段为 0 检查 ---------- */
    last_detail = "zero_field";
    if (!node || !id || !code) {
        return MQTT_COMMAND_BAD_FORMAT;
    }

    /* ---------- 步骤 5：解析主题中的节点 ID ---------- */
    last_detail = "topic";
    if (!topic_node_id(topic, &topic_node)) {
        return MQTT_COMMAND_BAD_FORMAT;
    }

    /* ---------- 步骤 6：字段范围检查 ---------- */
    last_detail = "range";
    if (node > 255u || code > 255u) {
        return MQTT_COMMAND_BAD_FORMAT;
    }

    /* ---------- 步骤 7：主题节点与 JSON 节点一致性 ---------- */
    last_detail = "topic_node";
    if (node != topic_node) {
        return MQTT_COMMAND_BAD_TOPIC_NODE;
    }

    /* ---------- 步骤 8：节点是否在已知列表中 ---------- */
    last_detail = "known_node";
    for (index = 0u; index < node_count; ++index) {
        if (known_nodes[index] == node) {
            known = 1u;
        }
    }
    if (!known) {
        return MQTT_COMMAND_UNKNOWN_NODE;
    }

    /* ---------- 步骤 9：命令特定字段解析 ---------- */
    if (code == APP_CMD_SET_RELAY) {
        /*
         * 本版本只支持单路继电器命令：channel + state。
         * 如果 JSON 中同时出现 mask、states 或 value，则视为非法。
         */
        last_detail = "relay_fields";
        has_channel = json_u32(json, "channel", &channel);
        has_state   = json_u32(json, "state", &state);

        if (!has_channel || !has_state ||
            json_u32(json, "mask", &value) ||
            json_u32(json, "states", &value) ||
            json_u32(json, "value", &value) ||
            channel < 1u || channel > 4u || state > 1u ||
            !app_relay_encode((uint8_t)channel, (uint8_t)state, &value)) {
            return MQTT_COMMAND_BAD_RELAY;
        }
    } else if (code != APP_CMD_GET_RELAY_STATE) {
        /* 非继电器类命令：value 字段可选 */
        (void)json_u32(json, "value", &value);
    }

    /* ---------- 步骤 10：输出解析结果 ---------- */
    command->node  = (uint8_t)node;
    command->id    = id;
    command->code  = (uint8_t)code;
    command->value = value;
    last_detail    = "ok";
    return MQTT_COMMAND_OK;
}

/* ============================================================
 * 继电器状态 JSON 生成
 * ============================================================ */

/**
 * @brief 生成继电器状态的 JSON 字符串。
 *
 * 输出格式：
 *   {"node_id":x,"command_id":y,"relay1":a,"relay2":b,
 *    "relay3":c,"relay4":d,"state_mask":m}
 *
 * @param out         输出缓冲区
 * @param out_size    缓冲区大小
 * @param node        节点 ID
 * @param command_id  命令 ID
 * @param state_mask  继电器状态掩码（低 4 位有效）
 * @return 写入字符数；失败返回负值
 */
int mqtt_relay_state_json(char *out, size_t out_size, uint8_t node,
                          uint32_t command_id, uint8_t state_mask)
{
    uint8_t state = (uint8_t)(state_mask & 0x0Fu);

    if (!out || !out_size) {
        return -1;
    }

    return snprintf(out, out_size,
                    "{\"node_id\":%u,\"command_id\":%lu,\"relay1\":%u,"
                    "\"relay2\":%u,\"relay3\":%u,\"relay4\":%u,\"state_mask\":%u}",
                    node, (unsigned long)command_id,
                    (state >> 0) & 1u,
                    (state >> 1) & 1u,
                    (state >> 2) & 1u,
                    (state >> 3) & 1u,
                    state);
}