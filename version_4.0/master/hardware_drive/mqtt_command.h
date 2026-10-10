#ifndef MQTT_COMMAND_H
#define MQTT_COMMAND_H

#include <stddef.h>
#include <stdint.h>

#include "app_protocol.h"

/* ============================================================
 * MQTT 命令解析模块
 *
 * 负责将云端下发的 MQTT 命令（JSON 格式）解析为内部结构体，
 * 并校验节点、命令码、继电器参数等是否合法。
 *
 * 本模块为 F407 主机专用版本，仅支持单路继电器命令。
 * 若工程里误加入旧 F103 版本的 mqtt_command.c，会导致符号
 * 冲突或行为不一致，可用 mqtt_command_parser_version() 检查。
 * ============================================================ */

/* ============================================================
 * 数据结构
 * ============================================================ */

/**
 * @brief 解析后的 MQTT 命令结构体。
 */
typedef struct {
    uint8_t  node;    /* 目标节点 ID */
    uint8_t  code;    /* 命令码（APP_CMD_xxx） */
    uint32_t id;      /* 命令唯一 ID */
    uint32_t value;   /* 命令参数（继电器命令已编码为 value） */
} MqttCommand;

/* ============================================================
 * 解析结果码
 * ============================================================ */

typedef enum {
    MQTT_COMMAND_OK = 0,               /* 解析成功 */
    MQTT_COMMAND_BAD_FORMAT,           /* JSON 格式错误或缺少必要字段 */
    MQTT_COMMAND_UNKNOWN_NODE,         /* 目标节点不在已知节点列表中 */
    MQTT_COMMAND_BAD_TOPIC_NODE,       /* 主题中的节点 ID 与 JSON 不一致 */
    MQTT_COMMAND_BAD_RELAY,            /* 继电器参数无效 */
    MQTT_COMMAND_AMBIGUOUS_RELAY       /* 继电器参数同时存在多种形式，产生歧义 */
} MqttCommandResult;

/* ============================================================
 * 版本与诊断接口
 * ============================================================ */

/**
 * @brief 返回 F407 主机命令解析器的固定版本字符串。
 *
 * gateway.c 会引用此符号，防止 Keil 工程误加入旧 F103 版本的
 * mqtt_command.c。若打印出的版本不符合预期，说明工程里存在
 * 旧版文件残留。
 *
 * @return 版本字符串（常量，不需要释放）
 */
const char *mqtt_command_parser_version(void);

/**
 * @brief 返回最近一次解析停留的阶段结果。
 *
 * 仅用于串口定位格式问题，可能的取值包括：
 *   "not_run"        —— 尚未运行
 *   "arguments"      —— 参数校验阶段
 *   "json_structure" —— JSON 结构校验失败
 *   "node_id"        —— 解析 node_id 失败
 *   "command_id"     —— 解析 command_id 失败
 *   "code"           —— 解析 code 失败
 *   "zero_field"     —— 某关键字段为 0
 *   "topic"          —— 主题格式错误
 *   "range"          —— 字段超出允许范围
 *   "topic_node"     —— 主题节点与 JSON 节点不一致
 *   "known_node"     —— 节点不在已知列表中
 *   "relay_fields"   —— 继电器字段解析失败
 *   "ok"             —— 解析成功
 *
 * @return 阶段字符串（常量，不需要释放）
 */
const char *mqtt_command_last_detail(void);

/* ============================================================
 * 命令解析
 * ============================================================ */

/**
 * @brief 解析 MQTT 下行命令。
 *
 * 流程：
 *   1) 参数校验；
 *   2) JSON 结构校验；
 *   3) 提取 node_id、command_id、code；
 *   4) 字段为 0 检查；
 *   5) 解析主题中的节点 ID；
 *   6) 字段范围检查；
 *   7) 主题节点与 JSON 节点一致性检查；
 *   8) 节点是否在已知列表中；
 *   9) 继电器命令的参数编码；
 *  10) 输出解析结果。
 *
 * 解析失败时，可通过 mqtt_command_last_detail() 查询停留在哪一步。
 *
 * @param topic        消息主题（用于提取节点 ID）
 * @param json         消息载荷（JSON 格式）
 * @param known_nodes  已知节点 ID 列表
 * @param node_count   节点数量
 * @param command      输出参数，解析成功时填入命令内容
 * @return 解析结果码（MQTT_COMMAND_OK 表示成功）
 */
MqttCommandResult mqtt_command_parse(const char *topic, const char *json,
                                     const uint8_t *known_nodes,
                                     uint8_t node_count,
                                     MqttCommand *command);

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
 * @return 写入的字符数（不含结尾 '\0'）；失败返回负值。
 */
int mqtt_relay_state_json(char *out, size_t out_size, uint8_t node,
                          uint32_t command_id, uint8_t state_mask);

#endif /* MQTT_COMMAND_H */