#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H
#include "config.h"
#include <stdint.h>

#define APP_MAGIC             0x4E52u         /* 应用层魔数 */
#define APP_VERSION           1u              /* 应用层版本 */
#define APP_NETWORK_ID        0x2026u         /* 网络ID */
#define APP_MASTER_ID         0u              /* 主节点ID */
// #include "config.h"
#define APP_THIS_NODE_ID      APP_NODE_ID
#define APP_MAX_PAYLOAD       14u              /* 最大载荷大小 */
#define APP_HEADER_SIZE       16u              /* 帧头大小 */
#define APP_MAX_FRAME_SIZE    32u              /* 最大帧大小 */
#define APP_RADIO_MAX_FRAME   32u              /* 无线传输最大帧大小 */

enum {
    APP_MSG_POLL = 0x01,                        /* 主机请求最新快照*/
    APP_MSG_TELEMETRY = 0x02,                   /* 从机返回模拟遥测 */
    APP_MSG_COMMAND = 0x10,                     /* 主机下发命令 */
    APP_MSG_COMMAND_ACK = 0x11                  /* 从机返回应用层执行结果 */
};

enum {
    APP_CMD_SET_LED = 0x01,                     /* 设置LED */
    APP_CMD_SET_SAMPLE_PERIOD = 0x02,           /* 设置采样周期 */
    APP_CMD_FORCE_SAMPLE = 0x03,                /* 强制采样 */
    APP_CMD_DEBUG_DUMP = 0x04                   /* 调试 dump */
};

enum {
    APP_RESULT_OK = 0,                          /* 操作成功 */
    APP_RESULT_INVALID_PARAM = 1,               /* 无效参数 */
    APP_RESULT_UNSUPPORTED = 2,                 /* 不支持的操作 */
    APP_RESULT_BAD_FRAME = 3                    /* 错误的帧格式 */
};

#pragma pack(push, 1)
typedef struct {
    uint16_t magic;                             /* 应用层魔数 */
    uint8_t version;                            /* 版本 */
    uint8_t type;                               /* 消息类型 */
    uint16_t network_id;                        /* 网络ID */
    uint16_t src_id;                            /* 源节点ID */
    uint16_t dst_id;                            /* 目标节点ID */
    uint16_t boot_id;                           /* 启动ID */
    uint16_t seq;                               /* 序列号 */
    uint8_t payload_len;                        /* 载荷长度 */
    uint8_t flags;                              /* 标志位 */
} AppHeader;          

typedef struct {
    uint16_t request_seq;                      /* 请求序列号 */
    int16_t temperature_centi_c;               /* 温度，单位为0.01°C */
    uint16_t humidity_centi_rh;                /* 湿度，单位为0.01%RH */
    uint32_t pressure_pa;                      /* 压力，单位为Pa */
    uint16_t sample_age_ms;                    /* 采样年龄，单位为ms */
    uint8_t sensor_status;                     /* 传感器状态 */
    uint8_t led_state;                         /* LED状态 */
} AppTelemetry;

typedef struct {
    uint32_t command_id;                       /* 命令ID */
    uint8_t command;                           /* 命令类型 */
    uint8_t reserved;                          /* 保留 */
    uint32_t value;                            /* 命令值 */
} AppCommand;

typedef struct {
    uint32_t command_id;                       /* 命令ID */
    uint8_t result;                            /* 命令执行结果 */
    uint8_t command;
//    uint8_t result;
    uint32_t value;
} AppCommandAck;
#pragma pack(pop)

enum {
    SENSOR_AHT20_OK = 0x01,      /* AHT20传感器正常 */
    SENSOR_BMP280_OK = 0x02,    /* BMP280传感器正常 */ 
    SENSOR_AHT20_CRC_OK = 0x04,   /* AHT20数据CRC校验通过 */
    SENSOR_DATA_FRESH = 0x08,    /* 0x08表示数据是最近一次采样的结果，0表示数据是上次采样的结果。 */
    SENSOR_AHT20_ERROR = 0x10,   /* AHT20传感器错误 */
    SENSOR_BMP280_ERROR = 0x20,  /* BMP280传感器错误 */
    SENSOR_MASTER_LOST = 0x80    /* 主节点丢失 */
};

/* 计算应用层CRC-16/CCITT-FALSE。 */
uint16_t app_crc16(const uint8_t *data, uint8_t length);
/* 构造固定32字节帧，失败返回0。 */
uint8_t app_build_frame(uint8_t *out, uint8_t type, uint16_t src_id, uint16_t dst_id, uint16_t boot_id, uint16_t seq, const void *payload, uint8_t payload_len);
/* 验证帧格式和CRC，并返回帧头/载荷只读指针。 */
uint8_t app_validate_frame(const uint8_t *frame, uint8_t frame_len, const AppHeader **header, const uint8_t **payload);

#endif
