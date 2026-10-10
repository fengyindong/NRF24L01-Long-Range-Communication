#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>

/* ============================================================
 * 协议常量
 *
 * 应用层帧结构：帧头(16B) + 载荷(0~14B) + CRC(2B)，不足部分补零，
 * 最终统一打包成 32 字节定长帧，由 nRF24L01 发送。
 * ============================================================ */

#define APP_MAGIC             0x4E52u   /* 帧起始魔数，用于快速识别 */
#define APP_VERSION           1u        /* 协议版本号 */
#define APP_NETWORK_ID        0x2026u   /* 网络 ID，防止不同网络间串扰 */
#define APP_MASTER_ID         0u        /* 主机（网关）节点 ID，固定为 0 */

#include "config.h"

#define APP_THIS_NODE_ID      APP_NODE_ID  /* 本节点 ID，取自 config.h */

#define APP_MAX_PAYLOAD       14u       /* 应用层最大载荷长度（字节） */
#define APP_HEADER_SIZE       16u       /* 帧头长度（字节） */
#define APP_MAX_FRAME_SIZE    32u       /* 应用层最大帧长（帧头+载荷+CRC） */
#define APP_RADIO_MAX_FRAME   32u       /* 射频层固定帧长（nRF24 固定 32 字节） */

/* ============================================================
 * 消息类型
 * ============================================================ */

enum {
    APP_MSG_POLL         = 0x01,   /* 主机轮询从机，请求遥测数据 */
    APP_MSG_TELEMETRY    = 0x02,   /* 从机上报遥测数据 */
    APP_MSG_COMMAND      = 0x10,   /* 主机下发控制命令 */
    APP_MSG_COMMAND_ACK  = 0x11    /* 从机回复命令执行结果 */
};

/* ============================================================
 * 命令码
 * ============================================================ */

enum {
    APP_CMD_SET_LED           = 0x01,   /* 设置 LED 开关 */
    APP_CMD_SET_SAMPLE_PERIOD = 0x02,   /* 设置采样周期（毫秒） */
    APP_CMD_FORCE_SAMPLE      = 0x03,   /* 强制立即采样一次 */
    APP_CMD_DEBUG_DUMP        = 0x04,   /* 输出调试信息（nRF 寄存器、传感器状态） */

    /* 设置单路继电器：
     *   value 低 4 位选一路（必须为 2 的幂），
     *   bit8~11 对应位为目标状态（1=吸合，0=断开）。
     * 未选中的三路保持当前状态不变。 */
    APP_CMD_SET_RELAY         = 0x10,
    APP_CMD_GET_RELAY_STATE   = 0x11    /* 查询继电器当前状态 */
};

/* ============================================================
 * 命令执行结果码
 * ============================================================ */

enum {
    APP_RESULT_OK              = 0,   /* 执行成功 */
    APP_RESULT_INVALID_PARAM   = 1,   /* 参数无效 */
    APP_RESULT_UNSUPPORTED     = 2,   /* 不支持的命令 */
    APP_RESULT_BAD_FRAME       = 3    /* 帧格式或 CRC 错误 */
};

/* ============================================================
 * 协议帧结构体（1 字节对齐，确保与无线传输字节流一致）
 *
 * 必须使用 #pragma pack(1)，否则编译器可能插入填充字节，
 * 导致发送方和接收方的内存布局不一致。
 * ============================================================ */

#pragma pack(push, 1)

/* 通用帧头，所有消息均以此开头 */
typedef struct {
    uint16_t magic;        /* 魔数，固定为 APP_MAGIC */
    uint8_t  version;      /* 协议版本，固定为 APP_VERSION */
    uint8_t  type;         /* 消息类型（APP_MSG_xxx） */
    uint16_t network_id;   /* 网络 ID，固定为 APP_NETWORK_ID */
    uint16_t src_id;       /* 源节点 ID */
    uint16_t dst_id;       /* 目标节点 ID */
    uint16_t boot_id;      /* 节点启动序号（用于检测重启） */
    uint16_t seq;          /* 帧序列号（用于去重和请求响应匹配） */
    uint8_t  payload_len;  /* 载荷实际长度 */
    uint8_t  flags;        /* 标志位（预留，通常为 0） */
} AppHeader;

/* 遥测数据载荷（APP_MSG_TELEMETRY） */
typedef struct {
    uint16_t request_seq;         /* 对应主机轮询请求的序列号 */
    int16_t  temperature_centi_c; /* 温度，单位：0.01℃ */
    uint16_t humidity_centi_rh;   /* 湿度，单位：0.01%RH */
    uint32_t pressure_pa;         /* 气压，单位：Pa */
    uint16_t sample_age_ms;       /* 采样数据年龄（毫秒） */
    uint8_t  sensor_status;       /* 传感器状态位（SENSOR_xxx） */
    uint8_t  led_state;           /* LED 当前状态（0 灭，1 亮） */
} AppTelemetry;

/* 命令载荷（APP_MSG_COMMAND） */
typedef struct {
    uint32_t command_id;  /* 命令唯一 ID，用于去重和应答匹配 */
    uint8_t  command;     /* 命令码（APP_CMD_xxx） */
    uint8_t  reserved;    /* 保留字节，置 0 */
    uint32_t value;       /* 命令参数 */
} AppCommand;

/* 命令应答载荷（APP_MSG_COMMAND_ACK） */
typedef struct {
    uint32_t command_id;  /* 对应命令 ID */
    uint8_t  command;     /* 命令码 */
    uint8_t  result;      /* 执行结果（APP_RESULT_xxx） */
    uint32_t value;       /* 返回值或当前状态 */
} AppCommandAck;

#pragma pack(pop)

/* ============================================================
 * 传感器状态标志位（在 AppTelemetry.sensor_status 中按位或组合）
 * ============================================================ */

enum {
    SENSOR_AHT20_OK       = 0x01,  /* AHT20 温湿度传感器工作正常 */
    SENSOR_BMP280_OK      = 0x02,  /* BMP280 气压传感器工作正常 */
    SENSOR_AHT20_CRC_OK   = 0x04,  /* AHT20 数据 CRC 校验通过 */
    SENSOR_DATA_FRESH     = 0x08,  /* 当前数据为最新采集 */
    SENSOR_AHT20_ERROR    = 0x10,  /* AHT20 传感器异常 */
    SENSOR_BMP280_ERROR   = 0x20,  /* BMP280 传感器异常 */
    SENSOR_MASTER_LOST    = 0x80   /* 从机检测到主机通信丢失 */
};

/* ============================================================
 * 函数原型
 * ============================================================ */

/* 计算应用层 CRC-16/CCITT-FALSE（初始值 0xFFFF，多项式 0x1021） */
uint16_t app_crc16(const uint8_t *data, uint8_t length);

/* 构造固定 32 字节帧，失败返回 0 */
uint8_t app_build_frame(uint8_t *out, uint8_t type, uint16_t src_id,
                        uint16_t dst_id, uint16_t boot_id, uint16_t seq,
                        const void *payload, uint8_t payload_len);

/* 验证帧格式和 CRC，并返回帧头/载荷只读指针；成功返回 1，失败返回 0 */
uint8_t app_validate_frame(const uint8_t *frame, uint8_t frame_len,
                           const AppHeader **header, const uint8_t **payload);

/* 把单路继电器编号/状态编码为无线命令 value：
 *   channel = 1~4，on = 0/1 */
uint8_t app_relay_encode(uint8_t channel, uint8_t on, uint32_t *value);

/* 校验单路继电器 value 并计算执行后的 4 位状态：
 *   未选中的三路保持 current_state 原值 */
uint8_t app_relay_apply(uint8_t current_state, uint32_t value,
                        uint8_t *next_state);

#endif /* APP_PROTOCOL_H */