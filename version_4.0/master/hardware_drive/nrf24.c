#include "nrf24.h"
#include "board.h"

/* ============================================================
 * SPI 命令字
 * ============================================================ */

#define CMD_R_REGISTER    0x00u   /* 读寄存器 */
#define CMD_W_REGISTER    0x20u   /* 写寄存器 */
#define CMD_R_RX_PAYLOAD  0x61u   /* 读接收载荷 */
#define CMD_W_TX_PAYLOAD  0xA0u   /* 写发送载荷 */
#define CMD_FLUSH_TX      0xE1u   /* 清空发送 FIFO */
#define CMD_FLUSH_RX      0xE2u   /* 清空接收 FIFO */
#define CMD_NOP           0xFFu   /* 空操作，用于读取状态或填充 */

/* ============================================================
 * 寄存器地址
 * ============================================================ */

#define REG_CONFIG        0x00u   /* 配置寄存器 */
#define REG_EN_AA         0x01u   /* 自动应答使能 */
#define REG_EN_RXADDR     0x02u   /* 接收地址使能 */
#define REG_SETUP_AW      0x03u   /* 地址宽度设置 */
#define REG_SETUP_RETR    0x04u   /* 自动重发设置 */
#define REG_RF_CH         0x05u   /* 射频通道 */
#define REG_RF_SETUP      0x06u   /* 射频设置 */
#define REG_STATUS        0x07u   /* 状态寄存器 */
#define REG_RX_ADDR_P0    0x0Au   /* 接收通道 0 地址 */
#define REG_TX_ADDR       0x10u   /* 发送地址 */
#define REG_RX_PW_P0      0x11u   /* 接收通道 0 载荷宽度 */
#define REG_FIFO_STATUS   0x17u   /* FIFO 状态 */
#define REG_DYNPD         0x1Cu   /* 动态载荷使能 */
#define REG_FEATURE       0x1Du   /* 特性寄存器 */

/* ============================================================
 * 配置位定义
 * ============================================================ */

#define CONFIG_EN_CRC     0x08u   /* 使能 CRC */
#define CONFIG_CRCO       0x04u   /* CRC 编码方式：0=1字节，1=2字节 */
#define CONFIG_PWR_UP     0x02u   /* 上电 */
#define CONFIG_PRIM_RX    0x01u   /* 主接收模式 */

#define STATUS_RX_DR      0x40u   /* 接收数据就绪 */
#define STATUS_TX_DS      0x20u   /* 发送数据完成 */
#define STATUS_MAX_RT     0x10u   /* 达到最大重发次数 */

/* ============================================================
 * 射频参数（主机与所有从机必须保持一致）
 * ============================================================ */

#define NRF_RF_CHANNEL     76u    /* 射频通道 76（2476 MHz） */

#include "master_config.h"

#ifndef NRF_RF_SETUP_VALUE
#define NRF_RF_SETUP_VALUE 0x06u  /* 默认：1Mbps + 最大功率 */
#endif

#define NRF_RETR_VALUE     0x35u  /* ARD=1.5ms，ARC=3 次 */

/* ============================================================
 * 底层 SPI 操作
 * ============================================================ */

/**
 * @brief 发送单字节命令，返回命令阶段读到的 STATUS 寄存器值。
 */
static uint8_t command(uint8_t cmd)
{
    uint8_t status;

    board_nrf_csn(0u);
    status = board_spi1_transfer(cmd);
    board_nrf_csn(1u);
    return status;
}

/**
 * @brief 读取一个单字节寄存器。
 */
static uint8_t read_reg(uint8_t reg)
{
    uint8_t value;

    board_nrf_csn(0u);
    board_spi1_transfer((uint8_t)(CMD_R_REGISTER | (reg & 0x1Fu)));
    value = board_spi1_transfer(CMD_NOP);
    board_nrf_csn(1u);
    return value;
}

/**
 * @brief 写入一个单字节寄存器。
 */
static void write_reg(uint8_t reg, uint8_t value)
{
    board_nrf_csn(0u);
    board_spi1_transfer((uint8_t)(CMD_W_REGISTER | (reg & 0x1Fu)));
    board_spi1_transfer(value);
    board_nrf_csn(1u);
}

/**
 * @brief 写入一段连续数据（用于地址、TX 载荷等）。
 */
static void write_buf(uint8_t cmd, const uint8_t *data, uint8_t length)
{
    board_nrf_csn(0u);
    board_spi1_transfer(cmd);
    while (length--) {
        board_spi1_transfer(*data++);
    }
    board_nrf_csn(1u);
}

/**
 * @brief 读取一段连续数据（用于地址、RX 载荷等）。
 */
static void read_buf(uint8_t cmd, uint8_t *data, uint8_t length)
{
    board_nrf_csn(0u);
    board_spi1_transfer(cmd);
    while (length--) {
        *data++ = board_spi1_transfer(CMD_NOP);
    }
    board_nrf_csn(1u);
}

/* ============================================================
 * 公开接口
 * ============================================================ */

/**
 * @brief 清除 RX_DR、TX_DS、MAX_RT 三个中断标志。
 */
void nrf24_clear_irqs(void)
{
    write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
}

/**
 * @brief 初始化 nRF24L01 模块。
 *
 * 配置：2 字节 CRC、pipe0 自动应答、5 字节地址、通道 76、
 * 1Mbps + 最大功率、固定 32 字节载荷。初始化后芯片处于掉电模式，
 * 由后续 start_listening 或 send 切换模式。
 */
void nrf24_init(void)
{
    board_nrf_csn(1u);
    board_nrf_ce(0u);
    board_delay_us(150000u);   /* 等待模块上电稳定，150 ms */

    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO);
    write_reg(REG_EN_AA, 0x01u);           /* 仅 pipe0 自动应答 */
    write_reg(REG_EN_RXADDR, 0x01u);       /* 仅 pipe0 接收 */
    write_reg(REG_SETUP_AW, 0x03u);        /* 5 字节地址 */
    write_reg(REG_SETUP_RETR, NRF_RETR_VALUE);
    write_reg(REG_RF_CH, NRF_RF_CHANNEL);
    write_reg(REG_RF_SETUP, NRF_RF_SETUP_VALUE);
    write_reg(REG_FEATURE, 0x00u);         /* 关闭动态载荷等特性 */
    write_reg(REG_DYNPD, 0x00u);
    write_reg(REG_RX_PW_P0, 32u);          /* 固定 32 字节载荷 */
    command(CMD_FLUSH_RX);
    command(CMD_FLUSH_TX);
    nrf24_clear_irqs();
    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP);
    board_delay_us(2000u);                 /* 等待上电稳定，2 ms */
}

/**
 * @brief 检测模块是否存在且 SPI 通信正常。
 *
 * 通过两次 NOP 读取 STATUS，并对 RF_CH 进行写回测试。
 *
 * @return 1 正常；0 异常。
 */
uint8_t nrf24_is_present(void)
{
    uint8_t old     = read_reg(REG_RF_CH);
    uint8_t status  = command(CMD_NOP);
    uint8_t status2 = command(CMD_NOP);

    /* STATUS 保留位 bit7 必须为 0；0x00/0xFF 说明 SPI 异常 */
    if ((status == 0x00u || status == 0xFFu || (status & 0x80u)) &&
        (status2 == 0x00u || status2 == 0xFFu || (status2 & 0x80u))) {
        return 0u;
    }

    /* 对 RF_CH 写入 0x2A 并回读验证 */
    write_reg(REG_RF_CH, 0x2Au);
    if (read_reg(REG_RF_CH) != 0x2Au) return 0u;

    /* 对 RF_CH 写入 0x55 并回读验证 */
    write_reg(REG_RF_CH, 0x55u);
    if (read_reg(REG_RF_CH) != 0x55u) return 0u;

    /* 恢复原值并确认 */
    write_reg(REG_RF_CH, old);
    if (read_reg(REG_RF_CH) != old) return 0u;

    /* 自检过程可能改动配置，恢复关键参数 */
    write_reg(REG_SETUP_RETR, NRF_RETR_VALUE);
    write_reg(REG_RF_SETUP, NRF_RF_SETUP_VALUE);
    return 1u;
}

/**
 * @brief 公开单寄存器读取接口（调试用）。
 */
uint8_t nrf24_read_register(uint8_t reg)
{
    return read_reg(reg);
}

/**
 * @brief 读取 5 字节地址寄存器（调试用）。
 */
void nrf24_read_address(uint8_t reg, uint8_t address[5])
{
    read_buf((uint8_t)(CMD_R_REGISTER | (reg & 0x1Fu)), address, 5u);
}

/**
 * @brief 让模块进入接收模式，监听指定地址。
 *
 * @param address 5 字节接收地址（写入 pipe0）
 */
void nrf24_start_listening(const uint8_t address[5])
{
    board_nrf_ce(0u);

    /* 每次回到 PRX 都重写关键配置，避免兼容模块在 PTX 切换后丢失配置。 */
    write_reg(REG_EN_AA, 0x01u);
    write_reg(REG_EN_RXADDR, 0x01u);
    write_reg(REG_SETUP_AW, 0x03u);
    write_reg(REG_SETUP_RETR, NRF_RETR_VALUE);
    write_reg(REG_RF_CH, NRF_RF_CHANNEL);
    write_reg(REG_RF_SETUP, NRF_RF_SETUP_VALUE);
    write_reg(REG_RX_PW_P0, 32u);

    write_buf((uint8_t)(CMD_W_REGISTER | REG_RX_ADDR_P0), address, 5u);

    /* 某些兼容芯片用 TX_ADDR 参与 PRX 自动 ACK，和 RX 地址保持一致更稳妥。 */
    write_buf((uint8_t)(CMD_W_REGISTER | REG_TX_ADDR), address, 5u);

    /* PTX 切回 PRX 时丢弃旧数据，避免旧包阻塞新的响应。 */
    command(CMD_FLUSH_RX);
    nrf24_clear_irqs();

    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO |
                          CONFIG_PWR_UP | CONFIG_PRIM_RX);
    board_nrf_ce(1u);
    board_delay_us(150u);   /* 等待进入接收模式，150 us */
}

/**
 * @brief 从 RX FIFO 读取一包数据（固定 32 字节）。
 *
 * @param data   输出缓冲区，至少 32 字节
 * @param length 输出参数，返回读取长度（固定 32）
 * @return 1 成功；0 无数据或参数错误
 */
uint8_t nrf24_receive(uint8_t *data, uint8_t *length)
{
    uint8_t fifo;

    if (!data || !length) return 0u;

    fifo = read_reg(REG_FIFO_STATUS);
    if ((fifo & 0x01u) != 0u) return 0u;   /* RX FIFO 为空 */

    read_buf(CMD_R_RX_PAYLOAD, data, 32u);
    write_reg(REG_STATUS, STATUS_RX_DR);   /* 清除接收中断标志 */
    *length = 32u;
    return 1u;
}

/**
 * @brief 发送一包数据（固定 32 字节），等待硬件 ACK。
 *
 * @param address    目标节点 5 字节地址
 * @param data       待发送数据，长度必须为 32
 * @param length     数据长度（必须为 32）
 * @param timeout_ms 软件超时时间（毫秒）
 * @return 1 收到 ACK；0 失败或超时
 */
uint8_t nrf24_send(const uint8_t address[5], const uint8_t *data,
                   uint8_t length, uint32_t timeout_ms)
{
    uint32_t started;
    uint8_t  status;

    if (!address || !data || length != 32u) return 0u;

    board_nrf_ce(0u);
    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP);
    write_buf((uint8_t)(CMD_W_REGISTER | REG_TX_ADDR), address, 5u);
    write_buf((uint8_t)(CMD_W_REGISTER | REG_RX_ADDR_P0), address, 5u);

    /* 新一笔发送只保留当前载荷，避免残留 RX 状态干扰模式切换。 */
    command(CMD_FLUSH_RX);
    command(CMD_FLUSH_TX);
    nrf24_clear_irqs();
    write_buf(CMD_W_TX_PAYLOAD, data, length);

    board_delay_us(150u);   /* 等待 PTX 模式稳定，150 us */
    board_nrf_ce(1u);
    board_delay_us(15u);    /* CE 脉冲宽度至少 10 us */
    board_nrf_ce(0u);

    started = board_millis();
    do {
        status = read_reg(REG_STATUS);

        if (status & STATUS_TX_DS) {
            write_reg(REG_STATUS, STATUS_TX_DS);
            return 1u;
        }
        if (status & STATUS_MAX_RT) {
            write_reg(REG_STATUS, STATUS_MAX_RT);
            command(CMD_FLUSH_TX);
            return 0u;
        }
    } while ((uint32_t)(board_millis() - started) < timeout_ms);

    command(CMD_FLUSH_TX);
    return 0u;
}