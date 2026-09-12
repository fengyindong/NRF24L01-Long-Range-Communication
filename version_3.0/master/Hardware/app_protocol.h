#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>

/* ------------------------- 协议核心常量定义 ------------------------- */

/* 帧起始魔数，用于校验数据帧的起始 */
#define APP_MAGIC             0x4E52u

/* 协议版本号 */
#define APP_VERSION           1u

/* 网络ID，用于区分不同的无线网络 */
#define APP_NETWORK_ID        0x2026u

/* 网关（主机）节点ID */
#define APP_MASTER_ID         0u

/* 引入配置头文件 */
#include "config.h"

/* 当前节点ID（从机节点ID配置） */
#define APP_THIS_NODE_ID      APP_NODE_ID

/* 应用层最大载荷长度 */
#define APP_MAX_PAYLOAD       14u

/* 应用层帧头大小 */
#define APP_HEADER_SIZE       16u

/* 最大帧长度 */
#define APP_MAX_FRAME_SIZE    32u

/* 射频最大帧长度 */
#define APP_RADIO_MAX_FRAME   32u

/* ------------------------- 消息类型定义 ------------------------- */

/* 消息类型枚举，用于区分无线通信中的不同帧 */
enum {
    APP_MSG_POLL         = 0x01,  /* 主机轮询从机 */
    APP_MSG_TELEMETRY    = 0x02,  /* 从机上报遥测数据 */
    APP_MSG_COMMAND      = 0x10,  /* 主机下发命令 */
    APP_MSG_COMMAND_ACK  = 0x11   /* 从机回复命令确认 */
};

/* ------------------------- 命令类型定义 ------------------------- */

/* 主机下发的具体命令编号 */
enum {
    APP_CMD_SET_LED           = 0x01,  /* 设置LED状态 */
    APP_CMD_SET_SAMPLE_PERIOD = 0x02,  /* 设置采样周期 */
    APP_CMD_FORCE_SAMPLE      = 0x03,  /* 强制触发一次采样 */
    APP_CMD_DEBUG_DUMP        = 0x04,   /* 请求调试信息输出 */
    APP_CMD_SET_RELAY = 0x10,        /* value: bit0~3通道掩码，bit8~11对应目标状态 */
    APP_CMD_GET_RELAY_STATE = 0x11
};

/* ------------------------- 结果类型定义 ------------------------- */

/* 从机回复的执行结果状态码 */
enum {
    APP_RESULT_OK              = 0,  /* 执行成功 */
    APP_RESULT_INVALID_PARAM   = 1,  /* 参数无效 */
    APP_RESULT_UNSUPPORTED     = 2,  /* 不支持的命令 */
    APP_RESULT_BAD_FRAME       = 3   /* 帧格式错误 */
};

/* ------------------------- 协议结构体定义 ------------------------- */
#pragma pack(push, 1)

/* 通用帧头结构体，所有通信数据帧均以此开头 */
typedef struct {
    uint16_t magic;        /* 帧起始魔数，用于校验 */
    uint8_t  version;      /* 协议版本号 */
    uint8_t  type;         /* 消息类型（见消息类型定义） */
    uint16_t network_id;   /* 网络ID */
    uint16_t src_id;       /* 源节点ID */
    uint16_t dst_id;       /* 目标节点ID */
    uint16_t boot_id;      /* 节点启动序号（用于重启检测） */
    uint16_t seq;          /* 帧序列号 */
    uint8_t  payload_len;  /* 实际载荷长度 */
    uint8_t  flags;        /* 标志位（预留） */
} AppHeader;

/* 遥测数据载荷结构体（传感器数据帧） */
typedef struct {
    uint16_t request_seq;         /* 对应轮询请求的序列号 */
    int16_t  temperature_centi_c; /* 温度值（单位：0.01摄氏度） */
    uint16_t humidity_centi_rh;   /* 湿度值（单位：0.01%RH） */
    uint32_t pressure_pa;         /* 气压值（单位：Pa） */
    uint16_t sample_age_ms;       /* 采样数据年龄（毫秒） */
    uint8_t  sensor_status;       /* 传感器状态位 */
    uint8_t  led_state;           /* LED当前状态 */
} AppTelemetry;

/* 命令载荷结构体（主机下发的命令帧） */
typedef struct {
    uint32_t command_id;  /* 命令唯一ID（用于去重/应答） */
    uint8_t  command;     /* 命令编号 */
    uint8_t  reserved;    /* 保留字节 */
    uint32_t value;       /* 命令参数值 */
} AppCommand;

/* 命令确认载荷结构体（从机回复的应答帧） */
typedef struct {
    uint32_t command_id;  /* 对应命令ID */
    uint8_t  command;     /* 命令编号 */
    uint8_t  result;      /* 执行结果 */
    uint32_t value;       /* 返回值（预留） */
} AppCommandAck;

#pragma pack(pop)

/* ------------------------- 传感器状态位定义 ------------------------- */

/* 传感器状态位标志（用于指示各传感器工作情况） */
enum {
    SENSOR_AHT20_OK       = 0x01,  /* AHT20 温湿度传感器正常 */
    SENSOR_BMP280_OK      = 0x02,  /* BMP280 气压传感器正常 */
    SENSOR_AHT20_CRC_OK   = 0x04,  /* AHT20 CRC校验成功 */
    SENSOR_DATA_FRESH     = 0x08,  /* 数据为最新采集 */
    SENSOR_AHT20_ERROR    = 0x10,  /* AHT20 传感器异常 */
    SENSOR_BMP280_ERROR   = 0x20,  /* BMP280 传感器异常 */
    SENSOR_MASTER_LOST    = 0x80   /* 主机连接丢失 */
};

/* ------------------------- 函数原型声明 ------------------------- */

/* 
 * 功能：计算应用层数据的CRC-16/CCITT-FALSE校验值。
 * 参数：
 *   data   - 指向需要校验的数据缓冲区
 *   length - 数据长度
 * 返回：16位CRC校验值
 */
uint16_t app_crc16(const uint8_t *data, uint8_t length);

/* 
 * 功能：构造一个固定32字节的帧数据。
 * 参数：
 *   out         - 输出缓冲区（至少32字节）
 *   type        - 消息类型
 *   src_id      - 源节点ID
 *   dst_id      - 目标节点ID
 *   boot_id     - 启动序号
 *   seq         - 帧序列号
 *   payload     - 载荷数据指针
 *   payload_len - 载荷数据长度
 * 返回：成功返回帧长度，失败返回0
 */
uint8_t app_build_frame(uint8_t *out, uint8_t type, uint16_t src_id,  uint16_t dst_id, uint16_t boot_id, uint16_t seq, const void *payload, uint8_t payload_len);

/* 
 * 功能：验证帧的格式和CRC校验，并返回帧头与载荷的只读指针。
 * 参数：
 *   frame     - 待验证的帧数据指针
 *   frame_len - 帧数据长度
 *   header    - 输出帧头指针
 *   payload   - 输出载荷指针
 * 返回：校验通过返回1，失败返回0
 */
uint8_t app_validate_frame(const uint8_t *frame, uint8_t frame_len,
                           const AppHeader **header, const uint8_t **payload);


/* 把单路继电器编号/状态编码为无线命令value。 */
uint8_t app_relay_encode(uint8_t channel, uint8_t on, uint32_t *value);
/* 校验继电器value并计算执行后的4位状态。 */
uint8_t app_relay_apply(uint8_t current_state, uint32_t value,
                        uint8_t *next_state);
#endif  /* APP_PROTOCOL_H */

