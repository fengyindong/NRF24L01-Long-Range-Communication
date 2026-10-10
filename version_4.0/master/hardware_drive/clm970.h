#ifndef CLM970_H
#define CLM970_H

#include <stdint.h>

/* ============================================================
 * CLM970（4G/MQTT 模块）驱动接口
 *
 * 该模块封装了 CLM970 4G 模组的 AT 指令流程，包括：
 *   - 开机自检、SIM 卡与网络注册
 *   - PDP 上下文激活
 *   - MQTT 连接、订阅、发布、断开
 *   - 下行 PUBLISH 消息回调
 *   - 断线重连（指数退避）
 * ============================================================ */

/* ============================================================
 * 回调类型
 * ============================================================ */

/**
 * @brief MQTT 下行消息回调函数类型。
 *
 * 当模块收到服务器下发的 PUBLISH 消息时，驱动会调用此回调，
 * 将解析出的主题和消息内容交给上层业务处理。
 *
 * @param topic   消息主题（以 '\0' 结尾）
 * @param payload 消息内容（以 '\0' 结尾）
 */
typedef void (*Clm970MessageHandler)(const char *topic, const char *payload);

/* ============================================================
 * 函数原型
 * ============================================================ */

/**
 * @brief 初始化 CLM970 驱动。
 *
 * 内部会初始化发送队列、状态机、行缓冲区等资源，
 * 并注册下行消息回调。
 *
 * @param handler 收到 MQTT 下行消息时的回调函数，可为 NULL。
 */
void clm970_init(Clm970MessageHandler handler);

/**
 * @brief CLM970 主任务函数。
 *
 * 需要在主循环中反复调用，用于驱动内部状态机、解析模块返回的
 * URC/响应，并执行 MQTT 连接、订阅、发布等流程。
 *
 * 典型主循环：
 *   for (;;) {
 *       clm970_task();
 *       // 其他任务
 *   }
 */
void clm970_task(void);

/**
 * @brief 发布一条 MQTT 消息。
 *
 * 消息会先进入内部发送队列，由 clm970_task() 在 MQTT 连接就绪后
 * 依次发出。对于遥测/状态类主题，队列中已有的同主题消息会被
 * 覆盖为最新值（防止积压过期数据）。
 *
 * @param topic    消息主题
 * @param payload  消息内容
 * @param qos      QoS 等级（0、1、2）
 * @param retained 是否保留消息（0 = 不保留，1 = 保留）
 * @return 1 表示成功加入发送队列；0 表示失败（参数错误或队列满）。
 */
uint8_t clm970_publish(const char *topic, const char *payload,
                       uint8_t qos, uint8_t retained);

/**
 * @brief 查询当前 MQTT 是否已连接并处于就绪状态。
 *
 * @return 1 表示已连接且已订阅；0 表示未就绪。
 */
uint8_t clm970_is_connected(void);

/**
 * @brief 动态设置 MQTT 下行消息回调函数。
 *
 * 可用于运行过程中切换消息处理逻辑。
 *
 * @param handler 新的回调函数，可为 NULL。
 */
void clm970_set_message_handler(Clm970MessageHandler handler);

#endif /* CLM970_H */