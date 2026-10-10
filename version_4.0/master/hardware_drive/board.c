#include "board.h"
#include "master_config.h"

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

/* ============================================================
 * 配置常量
 * ============================================================ */

#define CLM_RX_RING_SIZE    512u        /* CLM970 串口接收环形缓冲区大小 */
#define BOARD_WAIT_TIMEOUT  1000000u    /* 硬件轮询等待超时计数 */

/* ============================================================
 * 全局状态
 * ============================================================ */

static volatile uint32_t g_millis;                          /* 系统毫秒计数 */
static volatile uint8_t  g_clm_rx_ring[CLM_RX_RING_SIZE];   /* CLM 接收环形缓冲 */
static volatile uint16_t g_clm_rx_head;                     /* 环形缓冲写指针 */
static volatile uint16_t g_clm_rx_tail;                     /* 环形缓冲读指针 */
static volatile uint16_t g_clm_rx_count;                    /* 环形缓冲有效字节数 */
static volatile uint32_t g_clm_rx_bytes;                    /* 累计接收字节数 */
static volatile uint32_t g_clm_uart_errors;                 /* 串口错误计数 */
static volatile uint8_t  g_board_error;                     /* 板级错误标志 */

/* ============================================================
 * 错误钩子与系统节拍
 * ============================================================ */

/**
 * @brief 板级错误钩子。
 *
 * 当发生串口/SPI 超时等严重错误时被调用。此处仅设置错误标志，
 * 可在此扩展 LED 报警、日志记录等。
 */
void board_error_hook(const char *message)
{
    (void)message;      /* 避免未使用参数告警 */
    g_board_error = 1u;
}

/* SysTick 每 1ms 调用一次，递增系统毫秒计数。 */
void SysTick_Handler(void)
{
    ++g_millis;
}

/* ============================================================
 * 串口底层操作
 * ============================================================ */

/**
 * @brief 向指定 USART 阻塞发送一个字节，带超时保护。
 *
 * @param usart USART 外设指针（USART1 或 USART2）
 * @param value 要发送的字节
 */
static void board_uart_putc(USART_TypeDef *usart, uint8_t value)
{
    uint32_t timeout = BOARD_WAIT_TIMEOUT;

    /* 等待 TXE（发送寄存器空），带超时保护防止死等 */
    while (USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET) {
        if (--timeout == 0u) {
            board_error_hook("BOARD UART TX TIMEOUT\r\n");
            return;
        }
    }
    USART_SendData(usart, value);
}

/* ============================================================
 * 调试串口（USART1）初始化
 * ============================================================ */

static void board_debug_init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;

    /* 使能 GPIOA 与 USART1 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* 配置 PA9(TX) / PA10(RX) 为复用功能 USART1 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    gpio.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    /* 配置 USART1 参数 */
    USART_StructInit(&usart);
    usart.USART_BaudRate = DEBUG_BAUD;
    usart.USART_Mode     = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
}

/* ============================================================
 * 系统时钟初始化
 * ============================================================ */

/**
 * @brief 时钟初始化失败时的处理：先初始化调试串口以便输出错误，再打印消息。
 */
static void board_clock_failure(const char *message)
{
    board_debug_init();
    board_debug_write(message);
    board_error_hook(message);
}

/**
 * @brief 初始化系统时钟（HSE + PLL → 168MHz）。
 *
 * 配置步骤：
 *   1) 启用外部 8MHz 晶振 HSE；
 *   2) 配置电源主调压器为最高性能等级；
 *   3) 设置 Flash 等待周期（168MHz 对应 5 个周期）；
 *   4) 配置 AHB/APB1/APB2 总线分频；
 *   5) 配置并启动 PLL，等待锁定；
 *   6) 切换系统时钟源到 PLL 并确认；
 *   7) 更新 SystemCoreClock 并配置 1ms SysTick。
 *
 * @return 1 成功；0 失败。
 */
static uint8_t board_clock_init(void)
{
    uint32_t timeout;

    /* 复位 RCC，启用外部高速晶振 HSE */
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);

    if (RCC_WaitForHSEStartUp() != SUCCESS) {
        board_clock_failure("BOARD CLOCK ERROR: HSE\r\n");
        return 0u;
    }

    /* 使能电源管理时钟，配置主调压器为最高性能等级 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_MainRegulatorModeConfig(PWR_Regulator_Voltage_Scale1);

    /* Flash 等待周期必须匹配 168MHz */
    FLASH_SetLatency(FLASH_Latency_5);

    /* AHB = SYSCLK, APB1 = HCLK/4 (42MHz), APB2 = HCLK/2 (84MHz) */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div4);
    RCC_PCLK2Config(RCC_HCLK_Div2);

    /* PLL：HSE 8MHz / 8 * 336 / 2 = 168MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE, 8u, 336u, 2u, 7u);
    RCC_PLLCmd(ENABLE);

    /* 等待 PLL 锁定 */
    timeout = 100000u;
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
        if (--timeout == 0u) {
            board_clock_failure("BOARD CLOCK ERROR: PLL\r\n");
            return 0u;
        }
    }

    /* 切换系统时钟源为 PLL */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    timeout = 100000u;
    while (RCC_GetSYSCLKSource() != 0x08u) {
        if (--timeout == 0u) {
            board_clock_failure("BOARD CLOCK ERROR: SYSCLK\r\n");
            return 0u;
        }
    }

    /* PLL 参数固定为 8MHz / 8 * 336 / 2 = 168MHz；
     * 直接更新变量，避免依赖不同 CMSIS 版本的 SystemCoreClockUpdate 实现。 */
    SystemCoreClock = BOARD_SYSCLK_HZ;
    SysTick_Config(SystemCoreClock / 1000u);

    return 1u;
}

/* ============================================================
 * nRF24L01 接口初始化（SPI1 + GPIO）
 * ============================================================ */

static void board_nrf_gpio_init(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef  spi;

    /* 使能 GPIOA、GPIOB、SPI1 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /* 配置 SPI1 引脚：PA5=SCK, PA6=MISO, PA7=MOSI */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    gpio.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);

    /* 配置 nRF CSN：PA4 推挽输出，默认拉高 */
    gpio.GPIO_Pin   = GPIO_Pin_4;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    /* 配置 nRF CE：PB0 推挽输出，默认拉低 */
    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &gpio);

    GPIO_SetBits(GPIOA, GPIO_Pin_4);    /* CSN = 1 */
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);  /* CE  = 0 */

    /* 配置 SPI1：全双工主机，Mode 0，MSB 优先，32 分频 */
    SPI_I2S_DeInit(SPI1);
    SPI_StructInit(&spi);
    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode              = SPI_Mode_Master;
    spi.SPI_DataSize          = SPI_DataSize_8b;
    spi.SPI_CPOL              = SPI_CPOL_Low;
    spi.SPI_CPHA              = SPI_CPHA_1Edge;
    spi.SPI_NSS               = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
}

/* ============================================================
 * CLM970 串口（USART2）初始化
 * ============================================================ */

static void board_clm_uart_init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    /* 使能 GPIOA 与 USART2 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* 配置 PA2(TX) / PA3(RX) 为复用功能 USART2 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    gpio.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    /* 配置 USART2 参数 */
    USART_StructInit(&usart);
    usart.USART_BaudRate = CLM_BAUD;
    usart.USART_Mode     = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &usart);
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* 配置 USART2 接收中断（优先级 2,0） */
    nvic.NVIC_IRQChannel                   = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2u;
    nvic.NVIC_IRQChannelSubPriority        = 0u;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

/* ============================================================
 * 公共接口
 * ============================================================ */

/**
 * @brief 板级初始化总入口：时钟、nRF 接口、调试串口、CLM 串口。
 *
 * @return 1 成功；0 失败（通常是时钟初始化失败）。
 */
uint8_t board_init(void)
{
    /* 清零全局状态 */
    g_millis          = 0u;
    g_clm_rx_head     = 0u;
    g_clm_rx_tail     = 0u;
    g_clm_rx_count    = 0u;
    g_clm_rx_bytes    = 0u;
    g_clm_uart_errors = 0u;
    g_board_error     = 0u;

    if (!board_clock_init()) {
        return 0u;
    }

    board_nrf_gpio_init();
    board_debug_init();
    board_clm_uart_init();

    return 1u;
}

uint32_t board_millis(void)
{
    return g_millis;
}

/**
 * @brief 微秒级阻塞延时（粗略延时，主频 168MHz）。
 */
void board_delay_us(uint32_t us)
{
    while (us--) {
        volatile uint32_t cycles = 42u;  /* 经验值，视主频和优化等级调整 */
        while (cycles--) {
            __NOP();
        }
    }
}

/**
 * @brief SPI1 全双工交换一个字节，带超时保护。
 *
 * @param value 要发送的字节
 * @return 从 MISO 收到的字节；超时返回 0
 */
uint8_t board_spi1_transfer(uint8_t value)
{
    uint32_t timeout = BOARD_WAIT_TIMEOUT;

    /* 等待发送缓冲区为空 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
        if (--timeout == 0u) {
            board_error_hook("BOARD SPI TX TIMEOUT\r\n");
            return 0u;
        }
    }
    SPI_I2S_SendData(SPI1, value);

    /* 等待接收缓冲区非空 */
    timeout = BOARD_WAIT_TIMEOUT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
        if (--timeout == 0u) {
            board_error_hook("BOARD SPI RX TIMEOUT\r\n");
            return 0u;
        }
    }
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

/* nRF CSN 引脚控制（PA4） */
void board_nrf_csn(uint8_t high)
{
    if (high) {
        GPIO_SetBits(GPIOA, GPIO_Pin_4);
    } else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    }
}

/* nRF CE 引脚控制（PB0） */
void board_nrf_ce(uint8_t high)
{
    if (high) {
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
    } else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    }
}

/* 调试串口输出字符串 */
void board_debug_write(const char *text)
{
    if (!text) {
        return;
    }
    while (*text) {
        board_uart_putc(USART1, (uint8_t)*text++);
    }
}

/* CLM970 串口输出指定长度数据 */
void board_clm_write(const uint8_t *data, uint16_t length)
{
    if (!data) {
        return;
    }
    while (length--) {
        board_uart_putc(USART2, *data++);
    }
}

/**
 * @brief 从 CLM970 环形缓冲读取一个字节（非阻塞）。
 *
 * @param value 输出字节
 * @return 1 表示读到数据；0 表示缓冲为空
 */
uint8_t board_clm_read(uint8_t *value)
{
    uint16_t tail;

    if (!value || g_clm_rx_count == 0u) {
        return 0u;
    }

    tail   = g_clm_rx_tail;
    *value = g_clm_rx_ring[tail];
    g_clm_rx_tail = (uint16_t)((tail + 1u) % CLM_RX_RING_SIZE);
    --g_clm_rx_count;

    return 1u;
}

uint32_t board_clm_rx_bytes(void)
{
    return g_clm_rx_bytes;
}

uint32_t board_clm_uart_errors(void)
{
    return g_clm_uart_errors;
}

/* ============================================================
 * USART2 接收中断
 * ============================================================ */

/**
 * @brief CLM970 串口接收中断服务函数。
 *
 * 每收到一字节即存入环形缓冲区；发生错误时递增错误计数。
 * 注意：STM32F4 的 USART 状态位必须按“读 SR 后读 DR”清除，
 * 不能在检测错误标志时先另读一次 DR，否则会丢字节。
 */
void USART2_IRQHandler(void)
{
    uint16_t status = USART2->SR;
    uint8_t  value;
    uint16_t next;

    /* 无相关中断标志则直接返回 */
    if ((status & (USART_SR_RXNE | USART_SR_ORE |
                   USART_SR_FE   | USART_SR_NE | USART_SR_PE)) == 0u) {
        return;
    }

    value = (uint8_t)USART2->DR;   /* 读 DR 清除 RXNE 及错误标志 */

    if ((status & (USART_SR_ORE | USART_SR_FE |
                   USART_SR_NE  | USART_SR_PE)) != 0u) {
        ++g_clm_uart_errors;    /* 记录串口错误 */
    } else if ((status & USART_SR_RXNE) != 0u) {
        /* 计算下一个写位置，并检查缓冲区是否已满 */
        next = (uint16_t)((g_clm_rx_head + 1u) % CLM_RX_RING_SIZE);
        if (g_clm_rx_count < CLM_RX_RING_SIZE) {
            g_clm_rx_ring[g_clm_rx_head] = value;
            g_clm_rx_head = next;
            ++g_clm_rx_count;
            ++g_clm_rx_bytes;
        }
    }
}