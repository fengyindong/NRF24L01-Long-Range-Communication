#include "i2c_bus.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/* ------------------------- 引脚定义 ------------------------- */
#define SCL GPIO_Pin_6   /* I2C 时钟线（SCL） */
#define SDA GPIO_Pin_7   /* I2C 数据线（SDA） */

/* ------------------------- 底层时序函数 ------------------------- */

/* 产生模拟I2C半周期延时；循环次数可按系统主频和示波器结果调整。 */
static void delay_i2c(void)
{
    volatile uint16_t n;
    for (n = 0; n < 90u; ++n) __NOP();
}

/* 设置SCL开漏电平：传1表示释放总线，由外部上拉形成高电平。 */
static void scl(uint8_t h)
{
    if (h) GPIO_SetBits(GPIOB, SCL);
    else GPIO_ResetBits(GPIOB, SCL);
    delay_i2c();
}

/* 设置SDA开漏电平：传1表示释放，传0表示主动拉低。 */
static void sda(uint8_t h)
{
    if (h) GPIO_SetBits(GPIOB, SDA);
    else GPIO_ResetBits(GPIOB, SDA);
    delay_i2c();
}

/* 读取SDA实际电平，用于接收数据、ACK和判断总线是否被占用。 */
static uint8_t sda_read(void)
{
    return GPIO_ReadInputDataBit(GPIOB, SDA) == Bit_SET;
}

/* 产生I2C START：SCL为高时SDA由高变低。 */
static void start_condition(void)
{
    sda(1);
    scl(1);
    sda(0);
    scl(0);
}

/* 产生I2C STOP：SCL为高时SDA由低变高。 */
static void stop_condition(void)
{
    sda(0);
    scl(1);
    sda(1);
}

/* ------------------------- 字节收发函数 ------------------------- */

/*
 * 功能：按MSB优先发送一个字节，并读取从设备ACK。
 * 返回：1表示从设备拉低SDA应答，0表示NACK或设备不在线。
 */
static uint8_t write_byte(uint8_t v)
{
    uint8_t i, ack;

    for (i = 0; i < 8u; ++i) {
        sda((v & 0x80u) != 0u);  /* 发送最高位 */
        scl(1);                  /* SCL高电平，从设备采样 */
        scl(0);                  /* SCL拉低，准备下一位 */
        v <<= 1;                 /* 移位到下一位 */
    }

    /* 发送完8位数据后，读取从设备的ACK位（第9个时钟） */
    sda(1);                      /* 释放SDA，等待从设备拉低 */
    scl(1);
    ack = (uint8_t)!sda_read();  /* 如果SDA被拉低（0），说明收到了ACK */
    scl(0);
    return ack;
}

/*
 * 功能：从SDA读取一个字节。
 * 参数：ack非0时主机在第9时钟发送ACK，0时发送NACK结束读取。
 */
static uint8_t read_byte(uint8_t ack)
{
    uint8_t i, v = 0;

    sda(1);                      /* 释放SDA，准备接收数据 */
    for (i = 0; i < 8u; ++i) {
        v <<= 1;
        scl(1);
        if (sda_read()) v |= 1u; /* 读取当前位 */
        scl(0);
    }

    /* 发送ACK或NACK */
    sda(ack ? 0u : 1u);
    scl(1);
    scl(0);
    sda(1);                      /* 释放SDA */
    return v;
}

/* ------------------------- 公共接口函数 ------------------------- */

/* 功能：将PB6/PB7初始化为开漏输出，释放总线并执行一次总线恢复。 */
void i2c_bus_init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    g.GPIO_Pin = SCL | SDA;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &g);

    GPIO_SetBits(GPIOB, SCL | SDA);
    (void)i2c_bus_recover();
}

/*
 * 功能：SDA被从设备卡低时输出最多9个SCL脉冲，再发送STOP释放总线。
 * 返回：1表示恢复后SDA为高，0表示总线仍被拉低。
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

/* 功能：向7位I2C地址连续写入length字节；成功返回1，任意NACK返回0。 */
uint8_t i2c_write(uint8_t address7, const uint8_t *data, uint8_t length)
{
    start_condition();
    if (!write_byte((uint8_t)(address7 << 1))) goto fail;

    while (length--) {
        if (!write_byte(*data++)) goto fail;
    }

    stop_condition();
    return 1;

fail:
    stop_condition();
    return 0;
}

/* 功能：向指定设备的8位寄存器写入一个字节，是i2c_write的便捷封装。 */
uint8_t i2c_write_reg(uint8_t address7, uint8_t reg, uint8_t value)
{
    uint8_t b[2] = {reg, value};
    return i2c_write(address7, b, 2u);
}

/* 功能：不带寄存器地址地连续读取数据，主要用于AHT20命令式接口。 */
uint8_t i2c_read(uint8_t address7, uint8_t *data, uint8_t length)
{
    uint8_t n = length;

    if (!n) return 0;

    start_condition();
    if (!write_byte((uint8_t)((address7 << 1) | 1u))) goto fail;

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

/*
 * 功能：写入寄存器地址后使用重复START连续读取，主要用于BMP280。
 * 返回：1表示地址和数据阶段均成功，0表示任意阶段未应答。
 */
uint8_t i2c_read_regs(uint8_t address7, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (!length) return 0;

    start_condition();
    if (!write_byte((uint8_t)(address7 << 1))) goto fail;
    if (!write_byte(reg)) goto fail;

    start_condition();  /* 重复起始信号 */
    if (!write_byte((uint8_t)((address7 << 1) | 1u))) goto fail;

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