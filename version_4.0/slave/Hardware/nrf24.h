#ifndef NRF24_H
#define NRF24_H

#include <stdint.h>

/* ============================================================
 * nRF24L01 无线模块驱动接口
 *
 * 该模块封装了 nRF24L01+ 的底层 SPI 操作（本工程使用 GPIO 模拟
 * SPI），对外提供以下功能：
 *   - 模块初始化与自检
 *   - 接收模式配置与数据读取
 *   - 发送数据并等待硬件 ACK
 *   - 寄存器与地址读取（调试用）
 *
 * 说明：
 *   - 所有收发均使用固定 32 字节载荷，主机与所有从机必须一致。
 *   - 射频参数（通道、速率、功率、地址）由 app_config.h /
 *     master_config.h 统一配置，主从双方必须完全一致。
 * ============================================================ */

/* ============================================================
 * 初始化与自检
 * ============================================================ */

/**
 * @brief 初始化 GPIO 模拟 SPI 和 nRF24L01 基础寄存器。
 *
 * 默认配置：
 *   - 2 字节 CRC
 *   - pipe0 自动应答
 *   - 5 字节地址
 *   - 固定 32 字节载荷
 *   - 射频参数取自 NRF_RF_CHANNEL / NRF_RF_SETUP_VALUE
 *
 * 初始化后芯片处于掉电模式，由后续 nrf24_start_listening 或
 * nrf24_send 切换模式。
 */
void nrf24_init(void);

/**
 * @brief 验证 nRF24L01 寄存器能否可靠读写。
 *
 * 通过两次 NOP 读取 STATUS，并对 RF_CH 进行多模式写回测试：
 * 写入 0x2A / 0x55 后回读，最后恢复原值。
 *
 * @return 1 表示寄存器可可靠读写；0 通常表示供电、接线、MCU
 *         或模块异常。
 */
uint8_t nrf24_is_present(void);

/* ============================================================
 * 接收模式
 * ============================================================ */

/**
 * @brief 使用指定 5 字节 pipe0 地址进入持续接收模式（PRX）。
 *
 * 内部会重写关键配置寄存器，避免兼容模块在 PTX 切换后丢失配置，
 * 同时清空 RX FIFO 和中断标志。
 *
 * @param address 5 字节接收地址（写入 pipe0）
 */
void nrf24_start_listening(const uint8_t address[5]);

/**
 * @brief 非阻塞尝试从 RX FIFO 读取一个固定 32 字节载荷。
 *
 * 内部先检查 FIFO_STATUS 的 RX_EMPTY 标志，有数据才读取，
 * 读完清 RX_DR 中断标志。
 *
 * @param data   输出缓冲区，至少 32 字节
 * @param length 输出参数，返回实际读取长度（固定为 32）
 * @return 1 表示成功读取；0 表示当前无数据
 */
uint8_t nrf24_receive(uint8_t *data, uint8_t *length);

/* ============================================================
 * 发送
 * ============================================================ */

/**
 * @brief 发送固定 32 字节载荷并等待硬件 ACK。
 *
 * 内部流程：
 *   1) 切换为 PTX 模式；
 *   2) 写入目标地址（TX_ADDR 和 RX_ADDR_P0）；
 *   3) 清空 FIFO、清中断；
 *   4) 写入 TX FIFO；
 *   5) CE 脉冲触发发送；
 *   6) 轮询 STATUS，等待 TX_DS 或 MAX_RT。
 *
 * 注意：返回 1 表示收到对端硬件 ACK，**不代表对端应用层已经处理
 *       了数据**。ACK 丢失时可能数据已到达对端但本方返回 0。
 *
 * @param address    目标节点 5 字节地址
 * @param data       待发送数据，长度必须为 32
 * @param length     数据长度（必须为 32）
 * @param timeout_ms 软件兜底超时（毫秒）
 * @return 1 表示收到对端硬件 ACK；0 表示失败或超时
 */
uint8_t nrf24_send(const uint8_t address[5], const uint8_t *data,
                   uint8_t length, uint32_t timeout_ms);

/* ============================================================
 * 中断与寄存器（调试用）
 * ============================================================ */

/**
 * @brief 清除 RX_DR、TX_DS 和 MAX_RT 三个中断标志。
 */
void nrf24_clear_irqs(void);

/**
 * @brief 读取一个寄存器，供诊断输出使用。
 *
 * 注意：此接口会触发一笔 SPI 事务，正式收发状态机中不宜频繁调用。
 *
 * @param reg 寄存器地址
 * @return 寄存器当前值
 */
uint8_t nrf24_read_register(uint8_t reg);

/**
 * @brief 读取一个 5 字节地址寄存器，便于核对 RX/TX 地址和字节序。
 *
 * 注意：此接口会触发一笔 SPI 事务，正式收发状态机中不宜频繁调用。
 *
 * @param reg     地址寄存器（如 REG_RX_ADDR_P0、REG_TX_ADDR）
 * @param address 输出参数，返回 5 字节地址
 */
void nrf24_read_address(uint8_t reg, uint8_t address[5]);

#endif /* NRF24_H */