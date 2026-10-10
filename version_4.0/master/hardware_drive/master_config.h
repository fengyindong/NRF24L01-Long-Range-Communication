#ifndef MASTER_CONFIG_H
#define MASTER_CONFIG_H

#include <stdint.h>

/* ============================================================
 * 串口波特率
 * ============================================================ */

#define CLM_BAUD     9600u      /* CLM970 模块 AT 串口波特率 */
#define DEBUG_BAUD   115200u    /* 调试串口（USART1）波特率 */

/* ============================================================
 * CLM970 蜂窝网络配置
 *
 * 注意：APN 必须按实际 SIM 卡运营商修改，否则无法激活 PDP。
 * ============================================================ */

/* 中国移动物联网卡 APN（广东区域），其他运营商请替换 */
#define CLM_APN                  "CMIOTGZSW.GD.MNC008.MCC460.GPRS"

/* 模块上电后等待启动完成的时间（毫秒） */
#define CLM_BOOT_DELAY_MS        3000u

/* 普通 AT 指令响应超时时间（毫秒） */
#define CLM_COMMAND_TIMEOUT_MS   5000u

/* MQTT 连接建立超时时间（毫秒） */
#define CLM_CONNECT_TIMEOUT_MS  20000u

/* MQTT 发布超时时间（毫秒） */
#define CLM_PUBLISH_TIMEOUT_MS  30000u

/* MQTT 首次重连间隔（毫秒），失败后指数退避 */
#define CLM_RECONNECT_MS        30000u

/* 指数退避最大间隔（毫秒） */
#define CLM_BACKOFF_MAX_MS      60000u

/* 网络未注册时的轮询间隔（毫秒） */
#define CLM_REG_POLL_MS          5000u

/* MQTT 连接状态定期检查间隔（毫秒） */
#define CLM_STATE_CHECK_MS      30000u

/* 调试输出级别：0=只打印关键状态/错误，1=打印所有 AT 回显 */
#define CLM_DEBUG_VERBOSE           0u

/* 固件版本号，仅用于日志显示。
 * r4 版本说明：遥测 JSON 增加了带单位的字符串字段。 */
#define GATEWAY_FW_VERSION          "f407-master-20261010-r4-telemetry-units"

/* ============================================================
 * nRF24L01 射频配置
 *
 * 注意：主机和所有从机必须使用完全相同的射频参数，否则无法通信。
 * 当前恢复为已验证的 1Mbps + 最大功率。
 * ============================================================ */

/* 射频设置：1Mbps + 最大功率（0x06 组合） */
#define NRF_RF_SETUP_VALUE          0x06u

#if NRF_RF_SETUP_VALUE != 0x06u
#error "STM32F407 gateway requires NRF_RF_SETUP_VALUE=0x06"
#endif

/* ============================================================
 * STM32F407ZGT6 时钟配置
 *
 * 最小系统板：8MHz HSE，经 PLL 倍频得到 168MHz 系统时钟。
 * ============================================================ */

#define BOARD_HSE_HZ       8000000u    /* 外部晶振频率 */
#define BOARD_SYSCLK_HZ  168000000u    /* 系统时钟（SYSCLK） */
#define BOARD_PCLK1_HZ    42000000u    /* APB1 总线时钟 */
#define BOARD_PCLK2_HZ    84000000u    /* APB2 总线时钟 */

/* 保证本工程源文件在包含 stm32f4xx.h 前使用正确的晶振值。
 * 若标准库默认值与实际硬件不一致，此处的检查会提前暴露问题。 */
#ifndef HSE_VALUE
#define HSE_VALUE BOARD_HSE_HZ
#elif HSE_VALUE != BOARD_HSE_HZ
#error "STM32F407ZGT6 board requires HSE_VALUE=8000000"
#endif

/* ============================================================
 * MQTT 服务器配置
 *
 * 按 EMQX Cloud 或实际 MQTT 服务器账号修改。
 * ============================================================ */

#define CLM_MQTT_HOST       "broker.emqx.io"      /* MQTT 服务器地址 */
#define CLM_MQTT_PORT       1883u                 /* MQTT 端口（明文） */
#define CLM_MQTT_CLIENT_ID  "gateway01_f407z"     /* 客户端 ID，必须唯一 */

/* broker.emqx.io 允许匿名连接；EMQX Cloud 必须填写控制台创建的账号。 */
#define CLM_MQTT_USER       ""
#define CLM_MQTT_PASSWORD   ""

#define CLM_MQTT_TLS        0u   /* 是否启用 TLS：0=关闭，1=开启 */
#define CLM_MQTT_PUB_QOS    1u   /* 发布 QoS 等级（0/1/2） */
#define CLM_MQTT_SUB_QOS    1u   /* 订阅 QoS 等级（0/1/2） */

/* ============================================================
 * 网关应用参数
 * ============================================================ */

/* MQTT 主题根路径，例如 environment/gateway01 */
#define GATEWAY_MQTT_ROOT "environment/gateway01"

/* 从机节点数量 */
#define GATEWAY_NODE_COUNT 1u

/* 命令队列长度（缓存待下发的云端命令） */
#define GATEWAY_COMMAND_QUEUE_SIZE 8u

/* MQTT 发布队列长度（缓存待上传的消息） */
#define GATEWAY_PUBLISH_QUEUE_SIZE 8u

/* 正常轮询周期（毫秒） */
#define GATEWAY_NORMAL_POLL_MS 30000u

/* 命令发送失败后的重试间隔（毫秒） */
#define GATEWAY_COMMAND_RETRY_MS 100u

/* 等待从机应用层响应的超时时间（毫秒） */
#define GATEWAY_APPLICATION_TIMEOUT_MS 150u

/* ============================================================
 * 从机节点 ID 列表
 *
 * 必须与每个从机的 APP_NODE_ID 一致，顺序即轮询顺序。
 * ============================================================ */

static const uint8_t gateway_node_ids[GATEWAY_NODE_COUNT] = { 1u };

#endif /* MASTER_CONFIG_H */
