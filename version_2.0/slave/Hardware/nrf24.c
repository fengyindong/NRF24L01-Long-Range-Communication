#include "nrf24.h"
#include "stm32f10x.h"
// #include "stm32f10x_gpio.h"
// #include "stm32f10x_rcc.h"

extern uint32_t millis(void);

/* ------------------------- 引脚定义 ------------------------- */

/* CE引脚：控制芯片进入接收或发送模式 */
#define NRF_CE_PORT   GPIOB
#define NRF_CE_PIN    GPIO_Pin_0

/* CSN引脚：SPI片选信号，低电平有效 */
#define NRF_CSN_PORT  GPIOA
#define NRF_CSN_PIN   GPIO_Pin_4

/* ------------------------- SPI命令定义 ------------------------- */

/* nRF24L01  SPI 命令字 */
#define CMD_R_REGISTER    0x00u   /* 读寄存器 */
#define CMD_W_REGISTER    0x20u   /* 写寄存器 */
#define CMD_R_RX_PAYLOAD  0x61u   /* 读接收载荷 */
#define CMD_W_TX_PAYLOAD  0xA0u   /* 写发送载荷 */
#define CMD_FLUSH_TX      0xE1u   /* 清空发送FIFO */
#define CMD_FLUSH_RX      0xE2u   /* 清空接收FIFO */
#define CMD_R_RX_PL_WID   0x60u   /* 读接收载荷宽度（预留） */
#define CMD_ACTIVATE      0x50u   /* 激活命令（预留） */
#define CMD_NOP           0xFFu   /* 空操作，用于读取状态或填充 */

/* ------------------------- 寄存器地址定义 ------------------------- */

/* nRF24L01  寄存器地址 */
#define REG_CONFIG        0x00u   /* 配置寄存器 */
#define REG_EN_AA         0x01u   /* 自动应答使能 */
#define REG_EN_RXADDR     0x02u   /* 接收地址使能 */
#define REG_SETUP_AW      0x03u   /* 地址宽度设置 */
#define REG_SETUP_RETR    0x04u   /* 自动重发设置 */
#define REG_RF_CH         0x05u   /* 射频通道 */
#define REG_RF_SETUP      0x06u   /* 射频设置 */
#define REG_STATUS        0x07u   /* 状态寄存器 */
#define REG_RX_ADDR_P0    0x0Au   /* 接收通道0地址 */
#define REG_RX_PW_P0      0x11u   /* 接收通道0载荷宽度 */
#define REG_TX_ADDR       0x10u   /* 发送地址 */
#define REG_FIFO_STATUS   0x17u   /* FIFO状态 */
#define REG_DYNPD         0x1Cu   /* 动态载荷使能 */
#define REG_FEATURE       0x1Du   /* 特性寄存器 */

/* ------------------------- 配置位定义 ------------------------- */

/* 配置寄存器（REG_CONFIG）相关位 */
#define CONFIG_EN_CRC     0x08u   /* 使能CRC校验 */
#define CONFIG_CRCO       0x04u   /* CRC编码方案（0=1字节，1=2字节） */
#define CONFIG_PWR_UP     0x02u   /* 上电模式 */
#define CONFIG_PRIM_RX    0x01u   /* 主接收模式 */

/* 状态寄存器（REG_STATUS）中断位 */
#define STATUS_RX_DR      0x40u   /* 接收数据就绪 */
#define STATUS_TX_DS      0x20u   /* 发送数据完成 */
#define STATUS_MAX_RT     0x10u   /* 达到最大重发次数 */

/* ------------------------- 射频参数定义 ------------------------- */

/* 远距离默认射频参数：250 kbps + 最大功率；主机和所有从机必须保持一致。 */
#define NRF_RF_CHANNEL        76u   /* 2476 MHz；先恢复已验证的工作频道 */
#define NRF_RF_SETUP_VALUE    0x06u /* 1 Mbps、PA=0 dBm；bit0保留位必须为0 */
#define NRF_RETR_VALUE        0x35u /* 自动重发：ARD=4ms，ARC=15次 */

/* ------------------------- 基础GPIO操作函数 ------------------------- */

/* CE拉低：停止接收或结束发送脉冲。 */
static void ce_low(void) 
{ 
    GPIO_ResetBits(NRF_CE_PORT, NRF_CE_PIN); 
}

/* CE拉高：在PRX模式进入持续接收，在PTX模式启动发送。 */
static void ce_high(void) 
{
     GPIO_SetBits(NRF_CE_PORT, NRF_CE_PIN); 
    
}

/* CSN拉低：开始一笔SPI命令事务。 */
static void csn_low(void) 
{ 
    GPIO_ResetBits(NRF_CSN_PORT, NRF_CSN_PIN); 
}

/* CSN拉高：结束当前SPI事务并让无线芯片执行命令。 */
static void csn_high(void) 
{ 
    GPIO_SetBits(NRF_CSN_PORT, NRF_CSN_PIN); 
}

/* ------------------------- 软件模拟SPI（Mode 0） ------------------------- */

/*
 * 故障排查阶段使用低速GPIO模拟SPI mode 0：
 * PA5=SCK, PA6=MISO, PA7=MOSI。
 * 这样可排除硬件SPI/NSS/MODF配置问题，确认物理接线后可再换回硬件SPI。
 */
/* 产生GPIO模拟SPI的位间延时，降低速率以提高长杜邦线调试可靠性。 */
static void spi_bit_delay(void)
{
    volatile uint8_t n;
    for (n = 0; n < 20u; ++n) 
    __NOP();
}

/*
 * 功能：使用PA5/PA6/PA7完成一字节SPI Mode 0全双工交换。
 * 参数value为发给nRF的字节；返回同一时钟期间从MISO采集的字节。
 */
static uint8_t spi_rw(uint8_t value)
{
    uint8_t input = 0u;
    uint8_t mask;
    for (mask = 0x80u; mask != 0u; mask >>= 1) {
        if (value & mask) GPIO_SetBits(GPIOA, GPIO_Pin_7);
        else GPIO_ResetBits(GPIOA, GPIO_Pin_7);
        spi_bit_delay();
        GPIO_SetBits(GPIOA, GPIO_Pin_5);   /* SCK拉高（上升沿采样） */
        spi_bit_delay();
        input <<= 1;
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) != Bit_RESET) input |= 1u;
        GPIO_ResetBits(GPIOA, GPIO_Pin_5); /* SCK拉低 */
        spi_bit_delay();
    }
    return input;
}

/* ------------------------- 底层寄存器操作 ------------------------- */

/* 功能：发送无参数nRF命令并返回命令阶段的STATUS寄存器快照。 */
static uint8_t command(uint8_t cmd)
{
    uint8_t status;
    csn_low(); 
    status = spi_rw(cmd); 
    csn_high();
    return status;
}

/* 功能：读取一个单字节nRF寄存器。 */
static uint8_t read_reg(uint8_t reg)
{
    uint8_t v;
    csn_low();
    spi_rw(CMD_R_REGISTER | (reg & 0x1Fu)); 
    v = spi_rw(CMD_NOP); 
    csn_high();
    return v;
}

/* 功能：向一个单字节nRF寄存器写值。 */
static void write_reg(uint8_t reg, uint8_t value)
{
    csn_low(); 
    spi_rw(CMD_W_REGISTER | (reg & 0x1Fu)); 
    spi_rw(value); 
    csn_high();
}

/* 功能：在一笔CSN事务中发送命令和连续数据，用于地址及TX载荷。 */
static void write_buf(uint8_t cmd, const uint8_t *data, uint8_t length)
{
    csn_low(); 
    spi_rw(cmd); 
    while (length--)
    {
    spi_rw(*data++); 
    }
    csn_high();
}

/* 功能：发送命令后连续读取数据，用于地址寄存器及RX载荷。 */
static void read_buf(uint8_t cmd, uint8_t *data, uint8_t length)
{
    csn_low(); 
    spi_rw(cmd);
     while (length--)
     {
       *data++ = spi_rw(CMD_NOP); 
     }
    csn_high();
}

/* 功能：提供CE脉冲和模式切换所需的微秒级保守延时。 */
static void delay_us(uint16_t us)
{
    volatile uint32_t n;
    /* 在72 MHz下刻意留有裕量；这里只用于CE和模式切换的最小时序。 */
    while (us--) {
        for (n = 0; n < 60u; ++n)   __NOP();
    }
}

/* ------------------------- 外部接口函数 ------------------------- */

/* 功能：向STATUS的三个中断位写1，清除RX_DR、TX_DS和MAX_RT。 */
void nrf24_clear_irqs(void)
{
    write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
}

/*
 * 功能：初始化GPIO模拟SPI和nRF基础寄存器。
 * 配置：频道76、1Mbps、硬件2字节CRC、pipe0自动ACK、固定32字节载荷。
 * 注意：保留100ms模块上电等待，适配PA+LNA模块与兼容芯片。
 */
void nrf24_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;  /* PA5=SCK, PA7=MOSI */
    GPIO_Init(GPIOA, &gpio);
    /* 上拉使模块未连接时稳定读为0xFF，便于诊断。 */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = GPIO_Pin_6;               /* PA6=MISO */
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = NRF_CSN_PIN;
    GPIO_Init(NRF_CSN_PORT, &gpio);
    gpio.GPIO_Pin = NRF_CE_PIN;
    GPIO_Init(NRF_CE_PORT, &gpio);
    GPIO_ResetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_7);
    csn_high(); 
    ce_low();
    /* PA+LNA模块上电浪涌较大，等待电源和晶振稳定 150 ms。 */
    {
        uint32_t started = millis();
        while ((uint32_t)(millis() - started) < 150u) {}
    }

    /* 配置基本寄存器 */
    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO);    /* 使能CRC，2字节 */
    /* 仅pipe0启用自动应答；主从必须保持一致，否则会出现NO_HW_ACK。 */
    write_reg(REG_EN_AA, 0x01u);
    write_reg(REG_EN_RXADDR, 0x01u);
    write_reg(REG_SETUP_AW, 0x03u);                        /* 5字节地址 */
    /* ARD=4 ms、ARC=15 次；远距离/穿墙时比短重试更稳。 */
    write_reg(REG_SETUP_RETR, NRF_RETR_VALUE);
    write_reg(REG_RF_CH, NRF_RF_CHANNEL);
    /* 250 kbps、最大输出功率，优先提高接收灵敏度和穿墙余量。 */
    write_reg(REG_RF_SETUP, NRF_RF_SETUP_VALUE);

    /* 联调阶段使用固定32字节载荷，避开部分兼容芯片的DPL/FEATURE问题。 */
    write_reg(REG_FEATURE, 0x00u);
    write_reg(REG_DYNPD, 0x00u);
    write_reg(REG_RX_PW_P0, 32u);
    command(CMD_FLUSH_RX);
    command(CMD_FLUSH_TX);
    nrf24_clear_irqs();
    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP);
    /* 等待2ms确保配置生效 */
    {
        uint32_t started = millis();
        while ((uint32_t)(millis() - started) < 2u) {}
    }
}

/*
 * 功能：通过STATUS合法性和RF_CH多模式写回测试验证SPI链路。
 * 返回：1表示寄存器可可靠读写；0通常表示供电、接线、MCU或模块异常。
 */
uint8_t nrf24_is_present(void)
{
    uint8_t old = read_reg(REG_RF_CH);
    uint8_t status = command(CMD_NOP);
    uint8_t status2 = command(CMD_NOP);
    /* STATUS保留位7必须为0，RX_P_NO复位/空FIFO时通常为111。 */
    if ((status == 0x00u || status == 0xFFu || (status & 0x80u)) && (status2 == 0x00u || status2 == 0xFFu || (status2 & 0x80u))) return 0;
    write_reg(REG_RF_CH, 0x2Au);
    if (read_reg(REG_RF_CH) != 0x2Au) return 0;
    write_reg(REG_RF_CH, 0x55u);
    if (read_reg(REG_RF_CH) != 0x55u) return 0;
    write_reg(REG_RF_CH, old);
    if (read_reg(REG_RF_CH) != old) return 0u;
    /* 自检会临时改写RF_CH，恢复后确保工作配置仍完整。 */
    write_reg(REG_SETUP_RETR, NRF_RETR_VALUE);
    write_reg(REG_RF_SETUP, NRF_RF_SETUP_VALUE);
    return 1u;
}

/* 功能：公开单寄存器读取接口，供PA9调试输出使用。 */
uint8_t nrf24_read_register(uint8_t reg)
{
    return read_reg(reg);
}

/* 功能：读取5字节地址寄存器，便于核对RX/TX地址和字节序。 */
void nrf24_read_address(uint8_t reg, uint8_t address[5])
{
    read_buf(CMD_R_REGISTER | (reg & 0x1Fu), address, 5u);
}

/*
 * 功能：配置pipe0接收地址，清中断，切换到PRX并持续监听。
 * 参数：address为5字节空中地址，必须与主机写地址一致。
 */
void nrf24_start_listening(const uint8_t address[5])
{
    ce_low();
    write_buf(CMD_W_REGISTER | REG_RX_ADDR_P0, address, 5u);
    nrf24_clear_irqs();
    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP | CONFIG_PRIM_RX);
    ce_high();
    delay_us(150u);  /* 等待进入接收模式 */
}

/*
 * 功能：轮询RX FIFO并读取一个固定32字节载荷。
 * 返回：1表示data/length已更新；0表示当前无包。
 */
uint8_t nrf24_receive(uint8_t *data, uint8_t *length)
{
    uint8_t status = read_reg(REG_STATUS);
    uint8_t fifo = read_reg(REG_FIFO_STATUS);
    if ((fifo & 0x01u) != 0u) return 0; /* RX_EMPTY */
    read_buf(CMD_R_RX_PAYLOAD, data, 32u);
    write_reg(REG_STATUS, STATUS_RX_DR);
    *length = 32u;
    return 1;
}

/*
 * 功能：切换PTX、写入目标地址和32字节载荷，等待TX_DS或MAX_RT。
 * 参数：timeout_ms是软件兜底超时；硬件自动重发由SETUP_RETR控制。
 * 返回：1表示收到对端硬件ACK，0表示MAX_RT、参数错误或超时。
 */
uint8_t nrf24_send(const uint8_t address[5], const uint8_t *data, uint8_t length, uint32_t timeout_ms)
{
    uint32_t started;
    uint8_t status;
    if (length != 32u) return 0;
    ce_low();
    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP);
    write_buf(CMD_W_REGISTER | REG_TX_ADDR, address, 5u);
    write_buf(CMD_W_REGISTER | REG_RX_ADDR_P0, address, 5u);
    command(CMD_FLUSH_TX);
    nrf24_clear_irqs();
    write_buf(CMD_W_TX_PAYLOAD, data, length);
    /* PRX -> Standby-I -> TX需要最多约130us，不能配置完就立即脉冲CE。 */
    delay_us(150u);
    ce_high();
    delay_us(15u); 
    ce_low();
    started = millis();
    do {
        status = read_reg(REG_STATUS);
        if (status & STATUS_TX_DS) 
        { write_reg(REG_STATUS, STATUS_TX_DS); 
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