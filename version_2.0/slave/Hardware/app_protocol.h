#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>

/* ================= 协议基础常量定义 ================= */

/* 帧起始魔数，用于校验数据帧的开始 */
#define APP_MAGIC             0x4E52u

/* 协议版本号，版本变更时递增 */
#define APP_VERSION           1u

/* 网络ID，用于区分不同的无线网络，防止串网 */
#define APP_NETWORK_ID        0x2026u

/* 主机（网关）的节点ID，固定为0 */
#define APP_MASTER_ID         0u

/* 引入配置文件，其中定义了APP_NODE_ID等节点参数 */
#include "config.h"

/* 当前节点的ID，直接取自配置文件中的APP_NODE_ID */
#define APP_THIS_NODE_ID      APP_NODE_ID

/* 应用层最大载荷长度（不包含帧头），单位：字节 */
#define APP_MAX_PAYLOAD       14u

/* 应用层帧头大小，单位：字节 */
#define APP_HEADER_SIZE       16u

/* 应用层最大帧长度（帧头+载荷），单位：字节 */
#define APP_MAX_FRAME_SIZE    32u

/* 射频最大帧长度，必须与nRF24L01的固定载荷长度一致 */
#define APP_RADIO_MAX_FRAME   32u

/* ================= 消息类型定义 ================= */

/* 无线帧类型枚举 */
enum {
    APP_MSG_POLL         = 0x01,  /* 主机轮询从机请求数据 */
    APP_MSG_TELEMETRY    = 0x02,  /* 从机上报遥测数据（温湿度、气压等） */
    APP_MSG_COMMAND      = 0x10,  /* 主机下发控制命令 */
    APP_MSG_COMMAND_ACK  = 0x11   /* 从机回复命令执行结果 */
};

/* ================= 命令类型定义 ================= */

/* 主机下发的具体命令编号 */
enum {
    APP_CMD_SET_LED           = 0x01,  /* 设置LED开关状态 */
    APP_CMD_SET_SAMPLE_PERIOD = 0x02,  /* 设置传感器采样周期 */
    APP_CMD_FORCE_SAMPLE      = 0x03,  /* 强制触发一次立即采样 */
    APP_CMD_DEBUG_DUMP        = 0x04,  /* 请求从机输出调试信息 */
    APP_CMD_SET_RELAY         = 0x10   /* 设置继电器状态（value低4位为通道掩码，bit8~11对应目标状态） */
};

/* ================= 命令执行结果类型定义 ================= */

/* 从机回复的命令执行结果状态码 */
enum {
    APP_RESULT_OK              = 0,  /* 命令执行成功 */
    APP_RESULT_INVALID_PARAM   = 1,  /* 命令参数无效或越界 */
    APP_RESULT_UNSUPPORTED     = 2,  /* 不支持的命令类型 */
    APP_RESULT_BAD_FRAME       = 3   /* 帧格式错误或CRC校验失败 */
};

/* ================= 帧结构体定义 ================= */
#pragma pack(push, 1)  /* 强制1字节对齐，确保无线传输结构紧凑 */

/* 通用帧头结构体，所有通信帧均以此开头 */
typedef struct {
    uint16_t magic;        /* 帧起始魔数，用于快速校验 */
    uint8_t  version;      /* 协议版本号 */
    uint8_t  type;         /* 消息类型（见消息类型枚举） */
    uint16_t network_id;   /* 网络ID */
    uint16_t src_id;       /* 源节点ID */
    uint16_t dst_id;       /* 目标节点ID */
    uint16_t boot_id;      /* 节点启动序号（用于检测节点重启） */
    uint16_t seq;          /* 帧序列号（用于去重和排序） */
    uint8_t  payload_len;  /* 实际载荷长度 */
    uint8_t  flags;        /* 标志位（预留） */
} AppHeader;

/* 遥测数据载荷结构体（对应APP_MSG_TELEMETRY） */
typedef struct {
    uint16_t request_seq;         /* 对应主机的轮询请求序列号 */
    int16_t  temperature_centi_c; /* 温度值，单位：0.01摄氏度 */
    uint16_t humidity_centi_rh;   /* 湿度值，单位：0.01%RH */
    uint32_t pressure_pa;         /* 气压值，单位：帕斯卡（Pa） */
    uint16_t sample_age_ms;       /* 数据采样年龄（毫秒），距上次采集的时间 */
    uint8_t  sensor_status;       /* 传感器状态位（见传感器状态枚举） */
    uint8_t  led_state;           /* LED当前状态 */
} AppTelemetry;

/* 命令载荷结构体（对应APP_MSG_COMMAND） */
typedef struct {
    uint32_t command_id;  /* 命令唯一ID（用于去重和应答匹配） */
    uint8_t  command;     /* 命令编号（见命令类型枚举） */
    uint8_t  reserved;    /* 保留字节，暂未使用 */
    uint32_t value;       /* 命令参数值 */
} AppCommand;

/* 命令应答结构体（对应APP_MSG_COMMAND_ACK） */
typedef struct {
    uint32_t command_id;  /* 对应当前命令ID */
    uint8_t  command;     /* 命令编号 */
    uint8_t  result;      /* 执行结果（见结果状态枚举） */
    uint32_t value;       /* 返回值（预留，可存放状态） */
} AppCommandAck;

#pragma pack(pop)  /* 恢复默认对齐 */

/* ================= 传感器状态标志定义 ================= */

/* 传感器状态位标志（在AppTelemetry.sensor_status中按位或组合） */
enum {
    SENSOR_AHT20_OK       = 0x01,  /* AHT20温湿度传感器工作正常 */
    SENSOR_BMP280_OK      = 0x02,  /* BMP280气压传感器工作正常 */
    SENSOR_AHT20_CRC_OK   = 0x04,  /* AHT20传感器数据CRC校验通过 */
    SENSOR_DATA_FRESH     = 0x08,  /* 当前数据为最新采集，非缓存旧数据 */
    SENSOR_AHT20_ERROR    = 0x10,  /* AHT20传感器异常 */
    SENSOR_BMP280_ERROR   = 0x20,  /* BMP280传感器异常 */
    SENSOR_MASTER_LOST    = 0x80   /* 从机检测到主机通信丢失 */
};

/* ================= 函数原型声明 ================= */

/* 
 * 功能：计算应用层数据的CRC-16/CCITT-FALSE校验值。
 * 参数：
 *   data   - 指向需要计算的数据缓冲区
 *   length - 数据长度（字节）
 * 返回：16位CRC校验值
 */
uint16_t app_crc16(const uint8_t *data, uint8_t length);

/* 
 * 功能：构造一个固定32字节的无线数据帧。
 * 参数：
 *   out         - 输出缓冲区（至少APP_MAX_FRAME_SIZE字节）
 *   type        - 消息类型（见消息类型枚举）
 *   src_id      - 源节点ID
 *   dst_id      - 目标节点ID
 *   boot_id     - 节点启动序号
 *   seq         - 帧序列号
 *   payload     - 指向载荷数据的指针（可为NULL）
 *   payload_len - 载荷数据长度（不能超过APP_MAX_PAYLOAD）
 * 返回：成功返回帧总长度（一般为32），参数非法时返回0
 */
uint8_t app_build_frame(uint8_t *out, uint8_t type, uint16_t src_id,
                        uint16_t dst_id, uint16_t boot_id, uint16_t seq,
                        const void *payload, uint8_t payload_len);

/* 
 * 功能：验证接收到的帧格式和CRC校验。
 * 参数：
 *   frame     - 指向待验证的帧数据
 *   frame_len - 帧数据长度
 *   header    - 输出参数，返回帧头指针
 *   payload   - 输出参数，返回载荷指针
 * 返回：校验通过返回1，失败返回0
 */
uint8_t app_validate_frame(const uint8_t *frame, uint8_t frame_len,
                           const AppHeader **header, const uint8_t **payload);

#endif /* APP_PROTOCOL_H */