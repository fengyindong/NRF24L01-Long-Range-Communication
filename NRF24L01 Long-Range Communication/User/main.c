#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "config.h"
#include "app_protocol.h"
#include "nrf24.h"
#include "sensor.h"
#include <string.h>

static volatile uint32_t g_ms;                                          /* 全局单调毫秒计数 */
static const uint8_t MASTER_ADDRESS[5]={0xD2,0xD2,0xD2,0xD2,0xA0};      /* 主节点地址 */
static const uint8_t SLAVE_ADDRESS[5]={0xE7,0xE7,0xE7,0xE7,APP_NODE_ID}; /* 从节点地址 */   
static uint16_t boot_id,tx_seq;                               /* 启动ID和发送序列号 */
static uint8_t led_state;                                     /* LED状态 */
static uint32_t last_master_ms;                               /* 最后一次收到主节点消息的毫秒数 */
static uint8_t master_was_lost;                               /* 主节点是否丢失 */
static uint8_t cached_response[32],cached_response_len;       /* 缓存的响应数据和长度 */
static uint16_t cached_master_boot,cached_request_seq;         /* 缓存的主节点启动ID和请求序列号 */
static uint32_t last_command_id;                              /* 最后一个接收到的命令ID */
static AppCommandAck last_command_ack;                          /* 最后一个接收到的命令确认 */
static uint8_t last_command_valid;                            /* 最后一个接收到的命令是否有效 */
 
/* SysTick每1ms调用一次，维护全局单调毫秒计数。 */
void SysTick_Handler(void)
{
    ++g_ms;
}

/* 返回系统启动后的毫秒数；无符号减法可安全处理约49天回绕。 */
uint32_t millis(void)
{
    return g_ms;
}

/* 初始化USART1 TX(PA9)，用于输出无线、传感器和命令分层诊断。 */
static void debug_uart_init(void)
{
    GPIO_InitTypeDef g;
    USART_InitTypeDef u;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    g.GPIO_Pin = GPIO_Pin_9;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &g);

    USART_StructInit(&u);
    u.USART_BaudRate = 115200u;
    u.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &u);
    USART_Cmd(USART1, ENABLE);
}

/* 阻塞发送一个调试字符；仅在事件日志中调用，避免周期刷屏。 */
static void dbg_c(char c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART1, (uint16_t)c);
}

/* 输出以\0结尾的字符串；APP_DEBUG_ENABLED为0时直接返回。 */
static void dbg_s(const char *s)
{
    if (!APP_DEBUG_ENABLED)
    {
        return;
    }
    while (*s)
    {
        dbg_c(*s++);
    }
}

/* 以两位大写十六进制输出一个字节，便于阅读寄存器值。 */
static void dbg_hex(uint8_t v)
{
    static const char h[] = "0123456789ABCDEF";
    dbg_c(h[v >> 4]);
    dbg_c(h[v & 15]);
}

/* 不使用printf地输出uint32十进制，减小STM32 Flash占用。 */
static void dbg_u32(uint32_t v)
{
    char b[10];
    uint8_t n = 0;

    if (!v)
    {
        dbg_c('0');
        return;
    }

    while (v)
    {
        b[n++] = (char)('0' + v % 10u);
        v /= 10u;
    }

    while (n)
    {
        dbg_c(b[--n]);
    }
}

/* 读取关键nRF寄存器并输出，快速判断SPI、频道、速率和FIFO状态。 */
static void dbg_nrf(void)
{
    static const uint8_t r[] = {0x00, 0x03, 0x05, 0x06, 0x07, 0x11, 0x17, 0x1C, 0x1D};
    uint8_t i;

    dbg_s("NRF REG:");
    for (i = 0; i < sizeof(r); ++i)
    {
        dbg_c(' ');
        dbg_hex(r[i]);
        dbg_c('=');
        dbg_hex(nrf24_read_register(r[i]));
    }
    dbg_s("\r\n");
}

/* 初始化系统时钟计时、PC13 LED，并根据STM32 UID生成本次boot_id。 */
static void board_init(void)
{
    GPIO_InitTypeDef g;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    g.GPIO_Pin = GPIO_Pin_13;
    g.GPIO_Speed = GPIO_Speed_10MHz;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &g);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
    

    boot_id = (uint16_t)(*(volatile uint32_t *)0x1FFFF7E8u ^  *(volatile uint32_t *)0x1FFFF7ECu ^  SysTick->VAL);
}

/* 设置业务LED并保存逻辑状态；Blue Pill的PC13为低电平点亮。 */
static void led_set(uint8_t on)
{
    led_state = on ? 1u : 0u;
    if (led_state)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }
    else
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
}

/* nRF SPI自检失败后的致命状态：打印错误并快速闪烁，不进入业务循环。 */
static void fatal_radio(void)
{
    dbg_s("FATAL: NRF SPI\r\n");
    for (;;)
    {
        led_set((millis() / 100u) & 1u);
    }
}

/* 输出AHT20在线状态、BMP280地址和当前采样周期。 */
static void debug_sensor_status(void)
{
    dbg_s("SENSORS AHT=");
    dbg_u32(sensors_aht_address_ok());
    dbg_s(" BMP_ADDR=0x");
    dbg_hex(sensors_bmp_address());
    dbg_s(" PERIOD=");
    dbg_u32(sensors_get_period());
    dbg_s("ms\r\n");
}

/*
 * 功能：根据POLL或COMMAND生成32字节应用响应。
 * POLL返回最近完整传感器快照；COMMAND校验参数、执行并缓存结果。
 * 返回：成功为32，不支持的请求为0。
 */
static uint8_t build_response(const AppHeader *r, const uint8_t *p, uint8_t *out)
{
    if (r->type == APP_MSG_POLL && r->payload_len == 0u)
    {
        SensorSnapshot s;
        AppTelemetry t;
        uint32_t age;

        sensors_get_snapshot(&s);
        age = millis() - s.sampled_at_ms;
        t.request_seq = r->seq;
        t.temperature_centi_c = s.temperature_centi_c;
        t.humidity_centi_rh = s.humidity_centi_rh;
        t.pressure_pa = s.pressure_pa;
        t.sample_age_ms = (uint16_t)(age > 65535u ? 65535u : age);
        t.sensor_status = s.status;
        if (master_was_lost)
        {
            t.sensor_status |= SENSOR_MASTER_LOST;
        }
        t.led_state = led_state;

        return app_build_frame(out, APP_MSG_TELEMETRY, APP_NODE_ID, APP_MASTER_ID,
                               boot_id, ++tx_seq, &t, sizeof(t));
    }

    if (r->type == APP_MSG_COMMAND && r->payload_len == sizeof(AppCommand))
    {
        AppCommand c;
        AppCommandAck a;

        memcpy(&c, p, sizeof(c));

        if (last_command_valid && c.command_id == last_command_id)
        {
            dbg_s("CMD DUPLICATE id=");
            dbg_u32(c.command_id);
            dbg_s("\r\n");
            return app_build_frame(out, APP_MSG_COMMAND_ACK, APP_NODE_ID, APP_MASTER_ID,
                                   boot_id, ++tx_seq, &last_command_ack, sizeof(last_command_ack));
        }

        a.command_id = c.command_id;
        a.command = c.command;
        a.result = APP_RESULT_OK;
        a.value = c.value;

        switch (c.command)
        {
        case APP_CMD_SET_LED:
            if (c.value > 1u)
            {
                a.result = APP_RESULT_INVALID_PARAM;
            }
            else
            {
                led_set((uint8_t)c.value);
            }
            break;

        case APP_CMD_SET_SAMPLE_PERIOD:
            if (c.value < APP_MIN_SAMPLE_MS || c.value > APP_MAX_SAMPLE_MS)
            {
                a.result = APP_RESULT_INVALID_PARAM;
            }
            else
            {
                sensors_set_period(c.value);
                sensors_force_sample();
            }
            break;

        case APP_CMD_FORCE_SAMPLE:
            sensors_force_sample();
            a.value = 0u;
            break;

        case APP_CMD_DEBUG_DUMP:
            dbg_nrf();
            debug_sensor_status();
            a.value = 0u;
            break;

        default:
            a.result = APP_RESULT_UNSUPPORTED;
            break;
        }

        last_command_id = c.command_id;
        last_command_ack = a;
        last_command_valid = 1u;

        dbg_s("CMD id=");
        dbg_u32(c.command_id);
        dbg_s(" code=");
        dbg_u32(c.command);
        dbg_s(" result=");
        dbg_u32(a.result);
        dbg_s("\r\n");

        return app_build_frame(out, APP_MSG_COMMAND_ACK, APP_NODE_ID, APP_MASTER_ID,
                               boot_id, ++tx_seq, &a, sizeof(a));
    }

    return 0u;
}

/*
 * 功能：从机无线业务状态机，由主循环高频调用。
 * 流程：取包 -> 应用校验/身份检查 -> 去重 -> 构造响应 -> PTX发送 -> 恢复PRX。
 */
static void radio_task(void)
{
    uint8_t rx[32], n, response;
    const AppHeader *h;
    const uint8_t *p;

    if (!nrf24_receive(rx, &n))
    {
        return;
    }

    if (!app_validate_frame(rx, n, &h, &p))
    {
        dbg_s("RX DROP: CRC/FORMAT\r\n");
        return;
    }

    if (h->src_id != APP_MASTER_ID || h->dst_id != APP_NODE_ID)
    {
        dbg_s("RX DROP: ID\r\n");
        return;
    }

    master_was_lost = ((uint32_t)(millis() - last_master_ms) > APP_MASTER_LOST_MS) ? 1u : 0u;

    if (cached_response_len && h->boot_id == cached_master_boot && h->seq == cached_request_seq)
    {
        response = cached_response_len;
        dbg_s("RX DUPLICATE\r\n");
    }
    else
    {
        response = build_response(h, p, cached_response);
        if (!response)
        {
            return;
        }
        cached_response_len = response;
        cached_master_boot = h->boot_id;
        cached_request_seq = h->seq;
    }

    {
        uint32_t t = millis() + 2u;
        while ((int32_t)(millis() - t) < 0)
        {
        }
    }

    if (!nrf24_send(MASTER_ADDRESS, cached_response, response, 20u))
    {
        dbg_s("TX FAIL\r\n");
    }

    last_master_ms = millis();
    nrf24_start_listening(SLAVE_ADDRESS);
}

/* 程序入口：初始化调试、无线、传感器后永久运行采集和无线两个非阻塞任务。 */
int main(void)
{
    board_init();
    debug_uart_init();
    dbg_s("\r\nSTM32 NODE BOOT id=");
    dbg_u32(APP_NODE_ID);
    dbg_s("\r\n");
    led_set(0u);

    nrf24_init();
    dbg_nrf();
    if (!nrf24_is_present())
    {
        fatal_radio();
    }
    dbg_s("NRF PASS\r\n");

    sensors_init();
    debug_sensor_status();
    sensors_force_sample();
    last_master_ms = millis();
    nrf24_start_listening(SLAVE_ADDRESS);

    for (;;)
    {
        uint32_t now = millis();
        sensors_task(now);
        radio_task();
    }
}