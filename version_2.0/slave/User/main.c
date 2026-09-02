#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "config.h"
#include "app_protocol.h"
#include "nrf24.h"
#include "sensor.h"
#include <string.h>

/* ------------------------- 全局变量定义 ------------------------- */

/* 系统毫秒计数器，由SysTick中断每1ms递增 */
static volatile uint32_t g_ms;

/* 主机（网关）的5字节无线地址，必须与主机配置一致 */
static const uint8_t MASTER_ADDRESS[5] = {0xD2, 0xD2, 0xD2, 0xD2, 0xA0};

/* 本从机的5字节无线地址，前4字节固定，最后一个字节为节点ID */
static const uint8_t SLAVE_ADDRESS[5] = {0xE7, 0xE7, 0xE7, 0xE7, APP_NODE_ID};

/* 本次启动的ID，由STM32唯一ID与SysTick值异或生成，用于主机检测节点重启 */
static uint16_t boot_id;

/* 发送序号，每次发送响应帧时递增 */
static uint16_t tx_seq;

/* LED当前逻辑状态（1亮，0灭） */
static uint8_t led_state;

/* 4路继电器的当前状态位（bit0~bit3对应继电器1~4） */
static uint8_t relay_state;

/* 上一次收到主机帧的时间戳，用于判断主机是否丢失 */
static uint32_t last_master_ms;

/* 标记主机会话是否曾丢失（置1则上报遥测时带上SENSOR_MASTER_LOST标志） */
static uint8_t master_was_lost;

/* 缓存上次发给主机的完整响应帧，用于去重重传 */
static uint8_t cached_response[32];
static uint8_t cached_response_len;

/* 缓存上次主机的boot_id和请求seq，用于识别重复请求 */
static uint16_t cached_master_boot;
static uint16_t cached_request_seq;

/* 缓存最近一次命令ID及ACK数据，用于命令去重 */
static uint32_t last_command_id;
static AppCommandAck last_command_ack;
static uint8_t last_command_valid;

/* ------------------------- 系统节拍与时间获取 ------------------------- */

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

/* ------------------------- 调试串口（USART1）------------------------- */

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
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, (uint16_t)c);
}

/* 输出以\0结尾的字符串；APP_DEBUG_ENABLED为0时直接返回。 */
static void dbg_s(const char *s)
{
    if (!APP_DEBUG_ENABLED) return;
    while (*s) dbg_c(*s++);
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

    if (!v) {
        dbg_c('0');
        return;
    }
    while (v) {
        b[n++] = (char)('0' + v % 10u);
        v /= 10u;
    }
    while (n) dbg_c(b[--n]);
}

/* 读取关键nRF寄存器并输出，快速判断SPI、频道、速率和FIFO状态。 */
static void dbg_nrf(void)
{
    static const uint8_t r[] = {0x00, 0x03, 0x05, 0x06, 0x07, 0x11, 0x17, 0x1C, 0x1D};
    uint8_t i;

    dbg_s("NRF REG:");
    for (i = 0; i < sizeof(r); ++i) {
        dbg_c(' ');
        dbg_hex(r[i]);
        dbg_c('=');
        dbg_hex(nrf24_read_register(r[i]));
    }
    dbg_s("\r\n");
}

/* ------------------------- 板级初始化与输出控制 ------------------------- */

/* 初始化系统时钟计时、PC13 LED，并根据STM32 UID生成本次boot_id。 */
static void board_init(void)
{
    GPIO_InitTypeDef g;
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    g.GPIO_Pin = GPIO_Pin_13;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &g);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);

    /* 生成boot_id: 唯一的UID低位与SysTick当前值异或 */
    boot_id = (uint16_t)(*(volatile uint32_t*)0x1FFFF7E8u ^ *(volatile uint32_t*)0x1FFFF7ECu ^ SysTick->VAL);
}

/* 设置业务LED并保存逻辑状态；Blue Pill的PC13为低电平点亮。 */
static void led_set(uint8_t on)
{
    led_state = on ? 1u : 0u;
    if (led_state) GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    else GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/* 4路继电器预留：PB12~PB15，默认高电平关闭；如模块为低电平吸合请反相。 */
static void relay_set(uint8_t ch, uint8_t on)
{
    uint16_t pin;

    if (ch < 1u || ch > 4u) return;

    /* GPIO_Pin_11 << ch 巧妙地映射到 PB12~PB15 */
    pin = (uint16_t)(GPIO_Pin_11 << ch);
    if (on) {
        GPIO_ResetBits(GPIOB, pin);
        relay_state |= (uint8_t)(1u << (ch - 1u));
    } else {
        GPIO_SetBits(GPIOB, pin);
        relay_state &= (uint8_t)~(1u << (ch - 1u));
    }
}

/* 初始化继电器GPIO，默认全部关闭（高电平） */
static void relay_init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    g.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &g);
    GPIO_SetBits(GPIOB, g.GPIO_Pin);
    relay_state = 0u;
}

/* nRF SPI自检失败后的致命状态：打印错误并快速闪烁，不进入业务循环。 */
static void fatal_radio(void)
{
    dbg_s("FATAL: NRF SPI\r\n");
    for (;;) {
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

/* ------------------------- 响应帧构建 ------------------------- */

/*
 * 功能：根据POLL或COMMAND生成32字节应用响应。
 * POLL返回最近完整传感器快照；COMMAND校验参数、执行并缓存结果。
 * 返回：成功为32，不支持的请求为0。
 */
static uint8_t build_response(const AppHeader *r, const uint8_t *p, uint8_t *out)
{
    /* 处理主机发来的POLL（遥测请求） */
    if (r->type == APP_MSG_POLL && r->payload_len == 0u) {
        SensorSnapshot s;
        AppTelemetry t;
        uint32_t age;

        sensors_get_snapshot(&s);
        age = millis() - s.sampled_at_ms;

        /* 填充遥测数据包 */
        t.request_seq = r->seq;
        t.temperature_centi_c = s.temperature_centi_c;
        t.humidity_centi_rh = s.humidity_centi_rh;
        t.pressure_pa = s.pressure_pa;
        t.sample_age_ms = (uint16_t)(age > 65535u ? 65535u : age);
        t.sensor_status = s.status;

        /* 若检测到主机曾经丢失，则置位丢失标志 */
        if (master_was_lost) t.sensor_status |= SENSOR_MASTER_LOST;
        t.led_state = led_state;

        return app_build_frame(out, APP_MSG_TELEMETRY, APP_NODE_ID, APP_MASTER_ID, boot_id, ++tx_seq, &t, sizeof(t));
    }

    /* 处理主机发来的COMMAND（控制命令） */
    if (r->type == APP_MSG_COMMAND && r->payload_len == sizeof(AppCommand)) {
        AppCommand c;
        AppCommandAck a;
        memcpy(&c, p, sizeof(c));

        /* 命令去重：如果发现是上一次命令的重发，直接返回缓存的ACK */
        if (last_command_valid && c.command_id == last_command_id) {
            dbg_s("CMD DUPLICATE id=");
            dbg_u32(c.command_id);
            dbg_s("\r\n");
            return app_build_frame(out, APP_MSG_COMMAND_ACK, APP_NODE_ID, APP_MASTER_ID, boot_id, ++tx_seq, &last_command_ack, sizeof(last_command_ack));
        }

        /* 初始化响应结构体 */
        a.command_id = c.command_id;
        a.command = c.command;
        a.result = APP_RESULT_OK;
        a.value = c.value;

        /* 按命令类型执行 */
        switch (c.command) {
            case APP_CMD_SET_LED:
                if (c.value > 1u) a.result = APP_RESULT_INVALID_PARAM;
                else {
                    led_set((uint8_t)c.value);
                    a.value = led_state;
                    dbg_s("LED SET state=");
                    dbg_u32(led_state);
                    dbg_s("\r\n");
                }
                break;

            case APP_CMD_SET_RELAY: {
                /* value低4位(mask)选择通道，高4位(states)指定状态 */
                uint8_t mask = (uint8_t)(c.value & 0x0Fu);
                uint8_t states = (uint8_t)((c.value >> 8) & 0x0Fu);
                uint8_t i;

                if (!mask) a.result = APP_RESULT_INVALID_PARAM;
                else {
                    for (i = 1u; i <= 4u; i++) {
                        if (mask & (1u << (i - 1u))) {
                            relay_set(i, (states & (1u << (i - 1u))) ? 1u : 0u);
                        }
                    }
                    a.value = relay_state;
                    dbg_s("RELAY SET mask=");
                    dbg_u32(mask);
                    dbg_s(" state=");
                    dbg_u32(relay_state);
                    dbg_s("\r\n");
                }
            }
            break;

            case APP_CMD_SET_SAMPLE_PERIOD:
                /* 检查采样周期是否合法，若合法则更新并强制立即采样一次 */
                if (c.value < APP_MIN_SAMPLE_MS || c.value > APP_MAX_SAMPLE_MS) a.result = APP_RESULT_INVALID_PARAM;
                else {
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

        /* 缓存当前命令和ACK，用于后续去重 */
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
        return app_build_frame(out, APP_MSG_COMMAND_ACK, APP_NODE_ID, APP_MASTER_ID, boot_id, ++tx_seq, &a, sizeof(a));
    }
    return 0u;
}

/* ------------------------- 无线接收与响应任务 ------------------------- */

/*
 * 功能：从机无线业务状态机，由主循环高频调用。
 * 流程：取包 -> 应用校验/身份检查 -> 去重 -> 构造响应 -> PTX发送 -> 恢复PRX。
 */
static void radio_task(void)
{
    uint8_t rx[32], n, response;
    const AppHeader *h;
    const uint8_t *p;

    /* 1. 检查是否有新数据到达 */
    if (!nrf24_receive(rx, &n)) return;

    /* 2. 校验帧格式和CRC */
    if (!app_validate_frame(rx, n, &h, &p)) {
        dbg_s("RX DROP: CRC/FORMAT\r\n");
        return;
    }

    /* 3. 身份检查：只接受来自主机且发往本节点ID的帧 */
    if (h->src_id != APP_MASTER_ID || h->dst_id != APP_NODE_ID) {
        dbg_s("RX DROP: ID\r\n");
        return;
    }

    /* 4. 判断主机是否曾丢失（超过 APP_MASTER_LOST_MS 未收到） */
    master_was_lost = ((uint32_t)(millis() - last_master_ms) > APP_MASTER_LOST_MS) ? 1u : 0u;

    /* 5. 请求去重：如果主机的boot_id和seq都与上次相同，则重发缓存的响应 */
    if (cached_response_len && h->boot_id == cached_master_boot && h->seq == cached_request_seq) {
        response = cached_response_len;
        dbg_s("RX DUPLICATE\r\n");
    } else {
        /* 构造新响应 */
        response = build_response(h, p, cached_response);
        if (!response) return;
        cached_response_len = response;
        cached_master_boot = h->boot_id;
        cached_request_seq = h->seq;
    }

    /* 延时约2ms，给模块从RX切换到TX留出时间 */
    {
        uint32_t t = millis() + 2u;
        while ((int32_t)(millis() - t) < 0) {}
    }

    /* 6. 发送响应帧 */
    if (!nrf24_send(MASTER_ADDRESS, cached_response, response, 30u)) {
        dbg_s("TX FAIL\r\n");
    }

    /* 7. 更新最后收到主机帧的时间，并恢复到接收模式 */
    last_master_ms = millis();
    nrf24_start_listening(SLAVE_ADDRESS);
}

/* ------------------------- 程序入口 ------------------------- */

/* 程序入口：初始化调试、无线、传感器后永久运行采集和无线两个非阻塞任务。 */
int main(void)
{
    board_init();
    debug_uart_init();
    relay_init();

    dbg_s("\r\nSTM32 NODE BOOT id=");
    dbg_u32(APP_NODE_ID);
    dbg_s("\r\n");

    led_set(0);

    /* 初始化无线模块并进行自检 */
    nrf24_init();
    dbg_nrf();
    if (!nrf24_is_present()) fatal_radio();
    dbg_s("NRF PASS\r\n");

    /* 初始化传感器，强制立即采样一次 */
    sensors_init();
    debug_sensor_status();
    sensors_force_sample();

    last_master_ms = millis();
    nrf24_start_listening(SLAVE_ADDRESS);

    /* 主循环：非阻塞处理传感器采集任务和无线通信任务 */
    for (;;) {
        uint32_t now = millis();
        sensors_task(now);
        radio_task();
    }
}
