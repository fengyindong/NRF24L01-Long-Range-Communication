#include "nrf24.h"
#include "config.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

extern uint32_t millis(void);

/* ============================================================
 * 引脚定义
 * ============================================================ */

#define NRF_CE_PORT   GPIOB
#define NRF_CE_PIN    GPIO_Pin_0
#define NRF_CSN_PORT  GPIOA
#define NRF_CSN_PIN   GPIO_Pin_4

/* ============================================================
 * SPI 命令字
 * ============================================================ */

#define CMD_R_REGISTER    0x00u   /* 读寄存器 */
#define CMD_W_REGISTER    0x20u   /* 写寄存器 */
#define CMD_R_RX_PAYLOAD  0x61u   /* 读接收载荷 */
#define CMD_W_TX_PAYLOAD  0xA0u   /* 写发送载荷 */
#define CMD_FLUSH_TX      0xE1u   /* 清空发送 FIFO */
#define CMD_FLUSH_RX      0xE2u   /* 清空接收 FIFO */
#define CMD_R_RX_PL_WID   0x60u   /* 读接收载荷宽度（预留） */
#define CMD_ACTIVATE      0x50u   /* 激活命令（预留） */
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
#define REG_RX_PW_P0      0x11u   /* 接收通道 0 载荷宽度 */
#define REG_TX_ADDR       0x10u   /* 发送地址 */
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
 *
 * 已验证参数：1 Mbps + 最大功率。
 * 若主机与从机配置不一致，双方将完全无法通信。
 * ============================================================ */

#define NRF_RF_CHANNEL        76u   /* 射频通道 76（2476 MHz） */

#ifndef NRF_RF_SETUP_VALUE
#define NRF_RF_SETUP_VALUE    0x06u /* 1Mbps + 最大功率 */
#endif

#define NRF_RETR_VALUE        0x35u /* ARD=1.5ms，ARC=3 次 */

/* ============================================================
 * GPIO 控制函数
 * ============================================================ */

/* CE 拉低：停止接收或结束发送脉冲。 */
static void ce_low(void)
{
    GPIO_ResetBits(NRF_CE_PORT, NRF_CE_PIN);
}

/* CE 拉高：在 PRX 模式进入持续接收，在 PTX 模式启动发送。 */
static void ce_high(void)
{
    GPIO_SetBits(NRF_CE_PORT, NRF_CE_PIN);
}

/* CSN 拉低：开始一笔 SPI 命令事务。 */
static void csn_low(void)
{
    GPIO_ResetBits(NRF_CSN_PORT, NRF_CSN_PIN);
}

/* CSN 拉高：结束当前 SPI 事务并让无线芯片执行命令。 */
static void csn_high(void)
{
    GPIO_SetBits(NRF_CSN_PORT, NRF_CSN_PIN);
}

/* ============================================================
 * GPIO 模拟 SPI（Mode 0）
 *
 * 故障排查阶段使用低速 GPIO 模拟 SPI：
 *   PA5=SCK, PA6=MISO, PA7=MOSI。
 * 这样可排除硬件 SPI/NSS/MODF 配置问题，确认物理接线后可换回硬件 SPI。
 * ============================================================ */

/* 产生 GPIO 模拟 SPI 的位间延时，降低速率以提高长杜邦线调试可靠性。 */
static void spi_bit_delay(void)
{
    volatile uint8_t n;
    for (n = 0; n < 20u; ++n) __NOP();
}

/**
 * @brief 使用 PA5/PA6/PA7 完成一字节 SPI Mode 0 全双工交换。
 *
 * 时序（Mode 0，CPOL=0，CPHA=0，MSB 优先）：
 *   1) MOSI 输出当前位；
 *   2) SCK 上升沿（从机采样 MOSI，主机采样 MISO）；
 *   3) SCK 下降沿（准备下一位）。
 *
 * @param value 发给 nRF 的字节
 * @return 同一时钟期间从 MISO 采集的字节
 */
static uint8_t spi_rw(uint8_t value)
{
    uint8_t input = 0u;
    uint8_t mask;

    for (mask = 0x80u; mask != 0u; mask >>= 1) {
        /* MOSI 输出当前位 */
        if (value & mask) GPIO_SetBits(GPIOA, GPIO_Pin_7);
        else              GPIO_ResetBits(GPIOA, GPIO_Pin_7);
        spi_bit_delay();

        /* SCK 拉高（上升沿采样） */
        GPIO_SetBits(GPIOA, GPIO_Pin_5);
        spi_bit_delay();

        /* 采样 MISO */
        input <<= 1;
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) != Bit_RESET) {
            input |= 1u;
        }

        /* SCK 拉低 */
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);
        spi_bit_delay();
    }

    return input;
}

/* ============================================================
 * 底层寄存器操作
 * ============================================================ */

/* 发送无参数 nRF 命令并返回命令阶段的 STATUS 寄存器快照。 */
static uint8_t command(uint8_t cmd)
{
    uint8_t status;

    csn_low();
    status = spi_rw(cmd);
    csn_high();
    return status;
}

/* 读取一个单字节 nRF 寄存器。 */
static uint8_t read_reg(uint8_t reg)
{
    uint8_t v;

    csn_low();
    spi_rw(CMD_R_REGISTER | (reg & 0x1Fu));
    v = spi_rw(CMD_NOP);
    csn_high();
    return v;
}

/* 向一个单字节 nRF 寄存器写值。 */
static void write_reg(uint8_t reg, uint8_t value)
{
    csn_low();
    spi_rw(CMD_W_REGISTER | (reg & 0x1Fu));
    spi_rw(value);
    csn_high();
}

/* 在一笔 CSN 事务中发送命令和连续数据，用于地址及 TX 载荷。 */
static void write_buf(uint8_t cmd, const uint8_t *data, uint8_t length)
{
    csn_low();
    spi_rw(cmd);
    while (length--) {
        spi_rw(*data++);
    }
    csn_high();
}

/* 发送命令后连续读取数据，用于地址寄存器及 RX 载荷。 */
static void read_buf(uint8_t cmd, uint8_t *data, uint8_t length)
{
    csn_low();
    spi_rw(cmd);
    while (length--) {
        *data++ = spi_rw(CMD_NOP);
    }
    csn_high();
}

/* 提供 CE 脉冲和模式切换所需的微秒级保守延时。 */
static void delay_us(uint16_t us)
{
    volatile uint32_t n;

    /* 在 72 MHz 下刻意留有裕量；这里只用于 CE 和模式切换的最小时序。 */
    while (us--) {
        for (n = 0; n < 60u; ++n) __NOP();
    }
}

/* ============================================================
 * 公开接口
 * ============================================================ */

/* 向 STATUS 的三个中断位写 1，清除 RX_DR、TX_DS 和 MAX_RT。 */
void nrf24_clear_irqs(void)
{
    write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
}

/**
 * @brief 初始化 GPIO 模拟 SPI 和 nRF 基础寄存器。
 *
 * 配置：通道 76、1Mbps、硬件 2 字节 CRC、pipe0 自动应答、
 * 固定 32 字节载荷。初始化后芯片处于掉电模式。
 *
 * 注意：保留 150ms 模块上电等待，适配 PA+LNA 模块与兼容芯片。
 */
void nrf24_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* 配置 SCK(PA5)、MOSI(PA7) 为推挽输出 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);

    /* MISO(PA6) 上拉输入，模块未连接时稳定读为 0xFF，便于诊断 */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin  = GPIO_Pin_6;
    GPIO_Init(GPIOA, &gpio);

    /* CSN 与 CE 输出 */
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin  = NRF_CSN_PIN;
    GPIO_Init(NRF_CSN_PORT, &gpio);
    gpio.GPIO_Pin  = NRF_CE_PIN;
    GPIO_Init(NRF_CE_PORT, &gpio);

    GPIO_ResetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_7);
    csn_high();
    ce_low();

    /* PA+LNA 模块上电浪涌较大，等待电源和晶振稳定 150 ms。 */
    {
        uint32_t started = millis();
        while ((uint32_t)(millis() - started) < 150u) {}
    }

    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO);

    /* 仅 pipe0 启用自动应答；主从必须保持一致，否则会出现 NO_HW_ACK。 */
    write_reg(REG_EN_AA, 0x01u);
    write_reg(REG_EN_RXADDR, 0x01u);
    write_reg(REG_SETUP_AW, 0x03u);

    /* SETUP_RETR=0x35：ARD=1.5ms、ARC=3 次；主从必须完全一致。 */
    write_reg(REG_SETUP_RETR, NRF_RETR_VALUE);
    write_reg(REG_RF_CH, NRF_RF_CHANNEL);

    /* 1Mbps、最大输出功率，恢复之前稳定通信的配置。 */
    write_reg(REG_RF_SETUP, NRF_RF_SETUP_VALUE);

    /* 联调阶段使用固定 32 字节载荷，避开部分兼容芯片的 DPL/FEATURE 问题。 */
    write_reg(REG_FEATURE, 0x00u);
    write_reg(REG_DYNPD, 0x00u);
    write_reg(REG_RX_PW_P0, 32u);

    command(CMD_FLUSH_RX);
    command(CMD_FLUSH_TX);
    nrf24_clear_irqs();

    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP);

    {
        uint32_t started = millis();
        while ((uint32_t)(millis() - started) < 2u) {}
    }
}

/**
 * @brief 通过 STATUS 合法性和 RF_CH 多模式写回测试验证 SPI 链路。
 *
 * @return 1 表示寄存器可可靠读写；0 通常表示供电、接线、MCU 或模块异常。
 */
uint8_t nrf24_is_present(void)
{
    uint8_t old     = read_reg(REG_RF_CH);
    uint8_t status  = command(CMD_NOP);
    uint8_t status2 = command(CMD_NOP);

    /* STATUS 保留位 bit7 必须为 0；0x00/0xFF 说明 SPI 异常。 */
    if ((status == 0x00u || status == 0xFFu || (status & 0x80u)) &&
        (status2 == 0x00u || status2 == 0xFFu || (status2 & 0x80u))) {
        return 0;
    }

    /* 对 RF_CH 写入 0x2A 并回读验证 */
    write_reg(REG_RF_CH, 0x2Au);
    if (read_reg(REG_RF_CH) != 0x2Au) return 0;

    /* 对 RF_CH 写入 0x55 并回读验证 */
    write_reg(REG_RF_CH, 0x55u);
    if (read_reg(REG_RF_CH) != 0x55u) return 0;

    /* 恢复原值并确认 */
    write_reg(REG_RF_CH, old);
    if (read_reg(REG_RF_CH) != old) return 0u;

    /* 自检会临时改写 RF_CH，恢复后确保工作配置仍完整。 */
    write_reg(REG_SETUP_RETR, NRF_RETR_VALUE);
    write_reg(REG_RF_SETUP, NRF_RF_SETUP_VALUE);
    return 1u;
}

/* 公开单寄存器读取接口，供 PA9 调试输出使用。 */
uint8_t nrf24_read_register(uint8_t reg)
{
    return read_reg(reg);
}

/* 读取 5 字节地址寄存器，便于核对 RX/TX 地址和字节序。 */
void nrf24_read_address(uint8_t reg, uint8_t address[5])
{
    read_buf(CMD_R_REGISTER | (reg & 0x1Fu), address, 5u);
}

/**
 * @brief 配置 pipe0 接收地址，清中断，切换到 PRX 并持续监听。
 *
 * @param address 5 字节空中地址，必须与主机写地址一致。
 */
void nrf24_start_listening(const uint8_t address[5])
{
    ce_low();

    /* 每次回到 PRX 都重写关键配置，避免兼容模块在 PTX 切换后丢失配置。 */
    write_reg(REG_EN_AA, 0x01u);
    write_reg(REG_EN_RXADDR, 0x01u);
    write_reg(REG_SETUP_AW, 0x03u);
    write_reg(REG_SETUP_RETR, NRF_RETR_VALUE);
    write_reg(REG_RF_CH, NRF_RF_CHANNEL);
    write_reg(REG_RF_SETUP, NRF_RF_SETUP_VALUE);
    write_reg(REG_RX_PW_P0, 32u);

    write_buf(CMD_W_REGISTER | REG_RX_ADDR_P0, address, 5u);

    /* 某些兼容芯片用 TX_ADDR 参与 PRX 自动 ACK，和 RX 地址保持一致更稳妥。 */
    write_buf(CMD_W_REGISTER | REG_TX_ADDR, address, 5u);

    /* PTX 切回 PRX 时清掉残留 RX 载荷，确保下一次请求可正常接收。 */
    command(CMD_FLUSH_RX);
    nrf24_clear_irqs();

    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO |
                          CONFIG_PWR_UP | CONFIG_PRIM_RX);
    ce_high();
    delay_us(150u);
}

/**
 * @brief 轮询 RX FIFO 并读取一个固定 32 字节载荷。
 *
 * @return 1 表示 data/length 已更新；0 表示当前无包。
 */
uint8_t nrf24_receive(uint8_t *data, uint8_t *length)
{
    uint8_t fifo = read_reg(REG_FIFO_STATUS);

    if ((fifo & 0x01u) != 0u) return 0;   /* RX_EMPTY */

    read_buf(CMD_R_RX_PAYLOAD, data, 32u);
    write_reg(REG_STATUS, STATUS_RX_DR);
    *length = 32u;
    return 1;
}

/**
 * @brief 切换 PTX、写入目标地址和 32 字节载荷，等待 TX_DS 或 MAX_RT。
 *
 * @param address    目标节点 5 字节地址
 * @param data       待发送数据
 * @param length     数据长度（必须为 32）
 * @param timeout_ms 软件兜底超时（毫秒）
 * @return 1 收到对端硬件 ACK；0 表示 MAX_RT、参数错误或超时
 */
uint8_t nrf24_send(const uint8_t address[5], const uint8_t *data,
                   uint8_t length, uint32_t timeout_ms)
{
    uint32_t started;
    uint8_t  status;

    if (length != 32u) return 0;

    ce_low();
    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP);
    write_buf(CMD_W_REGISTER | REG_TX_ADDR, address, 5u);
    write_buf(CMD_W_REGISTER | REG_RX_ADDR_P0, address, 5u);

    /* 发送前丢弃旧 RX 载荷，避免模式切换后读到过期数据。 */
    command(CMD_FLUSH_RX);
    command(CMD_FLUSH_TX);
    nrf24_clear_irqs();
    write_buf(CMD_W_TX_PAYLOAD, data, length);

    /* PRX -> Standby-I -> TX 需要最多约 130us，不能配置完就立即脉冲 CE。 */
    delay_us(150u);
    ce_high();
    delay_us(15u);
    ce_low();

    started = millis();
    do {
        status = read_reg(REG_STATUS);

        if (status & STATUS_TX_DS) {
            write_reg(REG_STATUS, STATUS_TX_DS);
            return 1;
        }
        if (status & STATUS_MAX_RT) {
            write_reg(REG_STATUS, STATUS_MAX_RT);
            command(CMD_FLUSH_TX);
            return 0;
        }
    } while ((uint32_t)(millis() - started) < timeout_ms);

    command(CMD_FLUSH_TX);
    return 0;
}