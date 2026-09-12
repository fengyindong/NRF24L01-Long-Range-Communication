#ifndef MASTER_CONFIG_H
#define MASTER_CONFIG_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 网络连接配置（SIM卡及APN）                                          */
/* ------------------------------------------------------------------ */

 /* 运营商APN（接入点名称）——必须按你使用的SIM卡运营商修改，不要留空。 */
#define CLM_APN                 ""

/* 备用APN（当主APN为空时生效），与主APN相同，确保不会因空值导致连接失败 */
#define CLM_APN_FALLBACK        "CMIOTGZSW.GD.MNC008.MCC460.GPRS"

/* ------------------------------------------------------------------ */
/* MQTT 服务器连接配置                                                 */
/* ------------------------------------------------------------------ */

/* MQTT服务器地址（主机名或IP），可填公共测试服务器或EMQX Cloud控制台显示的Server */
#define MQTT_HOST               "broker.emqx.io"

/* MQTT服务器端口，1883为明文TCP端口，8883为TLS加密端口 */
#define MQTT_PORT               1883u

/* 是否启用TLS加密：0=关闭，1=开启（开启时需将端口改为TLS端口，如8883） */
#define MQTT_TLS                0u

/* MQTT用户名，公共服务器可留空，EMQX Cloud必须填写控制台生成的用户名 */
#define MQTT_USER               ""

/* MQTT密码，公共服务器可留空，EMQX Cloud必须填写控制台生成的密码 */
#define MQTT_PASSWORD           ""

/* MQTT客户端ID（必须唯一），若留空会自动使用默认兜底ID（见代码）。
 * 注意：必须与MQTTX等工具的Client ID不同，否则会导致互相踢下线。 */
#define MQTT_CLIENT_ID         ""

/* 发布消息的QoS等级：0=最多一次，1=至少一次，2=恰好一次 */
#define MQTT_PUB_QOS            0u

/* 订阅消息的QoS等级，同上 */
#define MQTT_SUB_QOS            0u

/* 网关标识符，用于主题命名，例如 environment/gateway01/... */
#define GATEWAY_ID              "gateway01"

/* ------------------------------------------------------------------ */
/* 串口通信参数                                                         */
/* ------------------------------------------------------------------ */

/* CLR970模块AT指令波特率，必须与实际模块一致。
 * 如果未执行AT+IPR=115200，则保持9600；若已修改波特率，则需同步修改。 */
#define CLM_BAUD                9600u

/* 模块上电后等待启动完成的时间（毫秒），确保模块初始化后再发送AT指令 */
#define CLM_BOOT_DELAY_MS       3000u

/* 调试串口（USART1）波特率，用于输出日志到PC */
#define DEBUG_BAUD              115200u

/* ------------------------------------------------------------------ */
/* 无线节点（nRF24）配置                                               */
/* ------------------------------------------------------------------ */

/* 从机节点数量（当前1个） */
#define NODE_COUNT              1u

/* 从机ID列表，必须与每个从机的APP_NODE_ID一致 */
static const uint8_t NODE_IDS[NODE_COUNT] = {1u};

/* nRF24射频信道（0~125），必须与从机nrf24.c中配置一致 */
#define RF_CHANNEL              76u

/* nRF24射频设置寄存器值（发射功率、速率等），详见nRF24手册 */
#define RF_SETUP_VALUE          0x06u

/* nRF24自动重传设置（重试次数、间隔） */
#define RF_RETR_VALUE           0x35u

/* ------------------------------------------------------------------ */
/* 节点状态轮询与故障判定                                               */
/* ------------------------------------------------------------------ */

/* 正常轮询间隔（毫秒），用于向从机请求遥测数据，可根据需要调整 */
#define NORMAL_POLL_MS          30000u

/* 离线节点探测间隔（毫秒），离线时以更频繁的间隔尝试恢复 */
#define OFFLINE_PROBE_MS        30000u

/* 连续失败多少次标记为“降级”状态 */
#define DEGRADED_FAILURES       3u

/* 连续失败多少次标记为“离线”状态 */
#define OFFLINE_FAILURES        5u

/* 连续成功多少次从离线恢复为在线 */
#define ONLINE_RECOVERY_OK      2u

/* ------------------------------------------------------------------ */
/* MQTT发布队列与超时参数                                               */
/* ------------------------------------------------------------------ */

/* MQTT发送队列长度，可容纳待发送的消息条数 */
#define MQTT_QUEUE_SIZE          8u

/* MQTT单条消息的最大负载长度（字节），AT指令最大长度需与之匹配 */
#define MQTT_PAYLOAD_MAX        320u

/* MQTT连接首次重连间隔（毫秒），之后采用指数退避 */
#define MQTT_RECONNECT_MS       30000u

/* 定期检查MQTT状态的时间间隔（毫秒） */
#define MQTT_STATE_CHECK_MS     30000u

/* MQTT发布超时（毫秒），超时后进入重连流程 */
#define MQTT_PUBLISH_TIMEOUT_MS 30000u

/* CLR970 AT指令的最大长度（字节），确保不超出模块接收缓冲 */
#define CLM_AT_LINE_MAX         320u/*  400u */

/* 普通AT指令响应超时（毫秒） */
#define CLM_COMMAND_TIMEOUT_MS  5000u

/* MQTT连接建立超时（毫秒） */
#define CLM_CONNECT_TIMEOUT_MS  20000u

/* 指数退避最大间隔（毫秒） */
#define CLM_BACKOFF_MAX_MS      60000u

/* 调试输出级别：0=只打印关键状态/错误，1=打印所有AT回显 */
#define DEBUG_VERBOSE           0u

/* 固件版本号，用于日志显示 */
#define GATEWAY_FW_VERSION      "App version 3.0.0"

/* MQTT主题根路径，通常格式为 environment/<gateway_id> */
#define MASTER_MQTT_ROOT        "environment/gateway01"

/* 是否启用OLED显示：1启用（需添加SSD1306 I2C驱动），0关闭（默认） */
#define OLED_ENABLED             0u

#endif /* MASTER_CONFIG_H */
