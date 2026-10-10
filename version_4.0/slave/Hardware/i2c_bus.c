#include "i2c_bus.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/* ============================================================
 * 引脚定义
 *
 * 使用 GPIOB 的 PB6/PB7 作为模拟 I2C：
 *   - PB6 = SCL（时钟线）
 *   - PB7 = SDA（数据线）
 * 均配置为开漏输出，外部必须接上拉电阻。
 * ============================================================ */

#define SCL GPIO_Pin_6
#define SDA GPIO_Pin_7

/* ============================================================
 * 底层时序函数
 * ============================================================ */

/* 产生模拟 I2C 半周期延时；循环次数可按系统主频和示波器结果调整。 */
static void delay_i2c(void)
{
    volatile uint16_t n;
    for (n = 0; n < 90u; ++n) __NOP();
}

/* 设置 SCL 开漏电平：传 1 表示释放总线，由外部上拉形成高电平。 */
static void scl(uint8_t h)
{
    if (h) GPIO_SetBits(GPIOB, SCL);
    else   GPIO_ResetBits(GPIOB, SCL);
    delay_i2c();
}

/* 设置 SDA 开漏电平：传 1 表示释放，传 0 表示主动拉低。 */
static void sda(uint8_t h)
{
    if (h) GPIO_SetBits(GPIOB, SDA);
    else   GPIO_ResetBits(GPIOB, SDA);
    delay_i2c();
}

/* 读取 SDA 实际电平，用于接收数据、ACK 和判断总线是否被占用。 */
static uint8_t sda_read(void)
{
    return GPIO_ReadInputDataBit(GPIOB, SDA) == Bit_SET;
}

/* 产生 I2C START：SCL 为高时 SDA 由高变低。 */
static void start_condition(void)
{
    sda(1);
    scl(1);
    sda(0);
    scl(0);
}

/* 产生 I2C STOP：SCL 为高时 SDA 由低变高。 */
static void stop_condition(void)
{
    sda(0);
    scl(1);
    sda(1);
}

/* ============================================================
 * 字节收发
 * ============================================================ */

/**
 * @brief 按 MSB 优先发送一个字节，并读取从设备 ACK。
 *
 * 时序：
 *   1) 循环 8 次，依次把 bit7~bit0 放到 SDA；
 *   2) 每次先拉高 SCL（从设备在上升沿采样），再拉低；
 *   3) 最后在第 9 个时钟周期读取 ACK：从设备拉低 SDA 表示应答。
 *
 * @param v 待发送的字节
 * @return 1 表示从设备拉低 SDA 应答；0 表示 NACK 或设备不在线
 */
static uint8_t write_byte(uint8_t v)
{
    uint8_t i, ack;

    /* 逐位发送 MSB 优先 */
    for (i = 0; i < 8u; ++i) {
        sda((v & 0x80u) != 0u);   /* 把当前最高位放到 SDA */
        scl(1);                   /* SCL 拉高，从设备采样 */
        scl(0);                   /* SCL 拉低，准备下一位 */
        v <<= 1;                  /* 数据左移，准备发送下一位 */
    }

    /* 第 9 个时钟：读取从设备的 ACK */
    sda(1);                       /* 释放 SDA，让从设备驱动 */
    scl(1);                       /* SCL 拉高 */
    ack = (uint8_t)!sda_read();   /* SDA 被拉低说明收到 ACK */
    scl(0);                       /* SCL 拉低，结束 ACK 周期 */

    return ack;
}

/**
 * @brief 从 SDA 读取一个字节。
 *
 * 时序：
 *   1) 循环 8 次，每次拉高 SCL 采样 SDA，再拉低；
 *   2) 第 9 个时钟周期发送 ACK 或 NACK。
 *
 * @param ack 非 0 时主机在第 9 时钟发送 ACK，0 时发送 NACK 结束读取
 * @return 读到的字节
 */
static uint8_t read_byte(uint8_t ack)
{
    uint8_t i, v = 0;

    sda(1);                       /* 释放 SDA，让从设备驱动 */

    /* 逐位读取 MSB 优先 */
    for (i = 0; i < 8u; ++i) {
        v <<= 1;
        scl(1);                   /* SCL 拉高，从设备输出数据 */
        if (sda_read()) v |= 1u;  /* 采样 SDA */
        scl(0);                   /* SCL 拉低，准备下一位 */
    }

    /* 第 9 个时钟：主机发送 ACK/NACK */
    sda(ack ? 0u : 1u);
    scl(1);
    scl(0);
    sda(1);                       /* 释放 SDA */

    return v;
}

/* ============================================================
 * 公共接口
 * ============================================================ */

/**
 * @brief 将 PB6/PB7 初始化为开漏输出，释放总线并执行一次总线恢复。
 *
 * 开漏输出是 I2C 的必要条件：
 *   - 写 1 时引脚高阻，由外部上拉电阻拉高；
 *   - 写 0 时引脚主动拉低。
 * 这样可以避免多个设备同时驱动总线时的电平冲突。
 */
void i2c_bus_init(void)
{
    GPIO_InitTypeDef g;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    g.GPIO_Pin   = SCL | SDA;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &g);

    GPIO_SetBits(GPIOB, SCL | SDA);   /* 释放总线 */
    (void)i2c_bus_recover();          /* 尝试恢复可能被卡死的总线 */
}

/**
 * @brief 总线恢复：SDA 被从设备卡低时输出最多 9 个 SCL 脉冲。
 *
 * 场景：如果主控在从设备传输过程中被复位，从设备可能一直把 SDA
 * 拉低，等待时钟。此时主机发送 9 个 SCL 脉冲，让从设备把剩余的
 * 位全部输出完，然后发 STOP 释放总线。
 *
 * @return 1 表示恢复后 SDA 为高；0 表示总线仍被拉低
 */
uint8_t i2c_bus_recover(void)
{
    uint8_t i;

    sda(1);
    for (i = 0; i < 9u && !sda_read(); ++i) {
        scl(0);
        scl(1);
    }
    stop_condition();

    return sda_read();
}

/**
 * @brief 向 7 位 I2C 地址连续写入 length 字节。
 *
 * 时序：START → 地址+W → 数据1 → 数据2 → ... → STOP
 *
 * @param address7 7 位从机地址（不含读写位）
 * @param data     待写入的数据
 * @param length   数据长度
 * @return 1 成功；0 表示任意阶段收到 NACK
 */
uint8_t i2c_write(uint8_t address7, const uint8_t *data, uint8_t length)
{
    start_condition();

    /* 发送地址 + 写位（最低位为 0） */
    if (!write_byte((uint8_t)(address7 << 1))) {
        goto fail;
    }

    /* 依次发送数据字节 */
    while (length--) {
        if (!write_byte(*data++)) {
            goto fail;
        }
    }

    stop_condition();
    return 1;

fail:
    stop_condition();
    return 0;
}

/**
 * @brief 向指定设备的 8 位寄存器写入一个字节。
 *
 * 是 i2c_write 的便捷封装，适用于 BMP280 等带寄存器地址的传感器。
 *
 * @param address7 7 位从机地址
 * @param reg      寄存器地址
 * @param value    要写入的值
 * @return 1 成功；0 失败
 */
uint8_t i2c_write_reg(uint8_t address7, uint8_t reg, uint8_t value)
{
    uint8_t b[2] = {reg, value};
    return i2c_write(address7, b, 2u);
}

/**
 * @brief 不带寄存器地址地连续读取数据。
 *
 * 主要用于 AHT20 命令式接口：先发测量命令，然后直接读回测量结果。
 *
 * 时序：START → 地址+R → 数据1（主机 ACK）→ ... → 数据n（主机 NACK）→ STOP
 *
 * @param address7 7 位从机地址
 * @param data     输出缓冲区
 * @param length   要读取的字节数
 * @return 1 成功；0 失败（参数错误或地址阶段 NACK）
 */
uint8_t i2c_read(uint8_t address7, uint8_t *data, uint8_t length)
{
    uint8_t n = length;

    if (!n) return 0;

    start_condition();

    /* 发送地址 + 读位（最低位为 1） */
    if (!write_byte((uint8_t)((address7 << 1) | 1u))) {
        goto fail;
    }

    /* 逐字节读取：除最后一个字节外，其余都发 ACK */
    while (n) {
        *data++ = read_byte(n > 1u);
        --n;
    }

    stop_condition();
    return 1;

fail:
    stop_condition();
    return 0;
}

/**
 * @brief 写入寄存器地址后使用重复 START 连续读取。
 *
 * 主要用于 BMP280 等需要先指定寄存器再读的传感器。
 *
 * 时序：START → 地址+W → 寄存器地址 → 重复 START → 地址+R → 数据1...数据n → STOP
 *
 * @param address7 7 位从机地址
 * @param reg      寄存器地址
 * @param data     输出缓冲区
 * @param length   要读取的字节数
 * @return 1 成功；0 表示任意阶段未应答
 */
uint8_t i2c_read_regs(uint8_t address7, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (!length) return 0;

    start_condition();

    /* 写阶段：发送地址 + 寄存器地址 */
    if (!write_byte((uint8_t)(address7 << 1))) {
        goto fail;
    }
    if (!write_byte(reg)) {
        goto fail;
    }

    /* 重复 START，切换到读阶段 */
    start_condition();
    if (!write_byte((uint8_t)((address7 << 1) | 1u))) {
        goto fail;
    }

    /* 逐字节读取：除最后一个字节外，其余都发 ACK */
    while (length) {
        *data++ = read_byte(length > 1u);
        --length;
    }

    stop_condition();
    return 1;

fail:
    stop_condition();
    return 0;
}