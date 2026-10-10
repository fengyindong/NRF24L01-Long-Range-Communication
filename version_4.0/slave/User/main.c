#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "config.h"
#include "app_protocol.h"
#include "nrf24.h"
#include "sensor.h"
#include <string.h>

/* ============================================================
 * 全局变量
 * ============================================================ */

static volatile uint32_t g_ms;                          /* SysTick 毫秒计数 */

static const uint8_t MASTER_ADDRESS[5] = {0xD2, 0xD2, 0xD2, 0xD2, 0xA0};    /* 主机地址 */
static const uint8_t SLAVE_ADDRESS[5]  = {0xE7, 0xE7, 0xE7, 0xE7, APP_NODE_ID}; /* 本机地址 */

static uint16_t boot_id, tx_seq;                        /* 启动 ID、发送序列号 */
static uint8_t  led_state;                              /* LED 逻辑状态 */
static uint8_t  relay_state;                            /* 4 路继电器状态（低 4 位） */
static uint32_t last_master_ms;                         /* 最近一次收到主机帧的时间 */
static uint8_t  master_was_lost;                        /* 主机是否曾丢失 */

/* 请求去重缓存：同一 boot_id + seq 的重发直接复用上次响应 */
static uint8_t  cached_response[32], cached_response_len;
static uint16_t cached_master_boot, cached_request_seq;

/* 命令去重缓存：只有 id + code + value 完全相同才认为是重发 */
static uint32_t      last_command_id;
static uint8_t       last_command_code;
static uint32_t      last_command_value;
static AppCommandAck last_command_ack;
static uint8_t       last_command_valid;

static uint8_t reset_flags;                             /* 本次启动的复位标志 */

/* 响应发送失败时的重试次数 */
#define RESPONSE_TX_RETRIES 3u

/* 4 路继电器对应的 GPIO 引脚（PB12~PB15） */
#define RELAY_PINS (GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15)

/* 复位来源标志位（与 RCC_CSR 寄存器对应） */
enum {
    RESET_CAUSE_PIN  = 0x01u,   /* NRST 引脚复位 */
    RESET_CAUSE_POR  = 0x02u,   /* 上电/掉电复位 */
    RESET_CAUSE_SW   = 0x04u,   /* 软件复位 */
    RESET_CAUSE_IWDG = 0x08u,   /* 独立看门狗复位 */
    RESET_CAUSE_WWDG = 0x10u,   /* 窗口看门狗复位 */
    RESET_CAUSE_LPWR = 0x20u    /* 低功耗管理复位 */
};

/* ============================================================
 * 系统节拍
 * ============================================================ */

/* SysTick 每 1ms 调用一次，维护全局单调毫秒计数。 */
void SysTick_Handler(void)
{
    ++g_ms;
}

/* 返回系统启动后的毫秒数；无符号减法可安全处理约 49 天回绕。 */
uint32_t millis(void)
{
    return g_ms;
}

/* ============================================================
 * 调试串口（USART1，PA9 TX）
 * ============================================================ */

/* 初始化 USART1 TX(PA9)，用于输出无线、传感器和命令分层诊断。 */
static void debug_uart_init(void)
{
    GPIO_InitTypeDef  g;
    USART_InitTypeDef u;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    g.GPIO_Pin   = GPIO_Pin_9;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &g);

    USART_StructInit(&u);
    u.USART_BaudRate = 115200u;
    u.USART_Mode     = USART_Mode_Tx;
    USART_Init(USART1, &u);
    USART_Cmd(USART1, ENABLE);
}

/* 阻塞发送一个调试字符；仅在事件日志中调用，避免周期刷屏。 */
static void dbg_c(char c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, (uint16_t)c);
}

/* 输出以 '\0' 结尾的字符串；APP_DEBUG_ENABLED 为 0 时直接返回。 */
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

/* 输出 16 位十六进制数，用于检查 PB12~PB15 实际输出电平。 */
static void dbg_hex16(uint16_t v)
{
    dbg_hex((uint8_t)(v >> 8));
    dbg_hex((uint8_t)v);
}

/* 不使用 printf 地输出 uint32 十进制，减小 STM32 Flash 占用。 */
static void dbg_u32(uint32_t v)
{
    char    b[10];
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

/* ============================================================
 * 复位来源诊断
 * ============================================================ */

/**
 * @brief 采集本次启动的复位来源标志。
 *
 * 必须在 RCC_ClearFlag() 之前调用，否则读不到标志。
 * STM32F1 的 PORRST 同时覆盖 POR/PDR/BOR 类供电复位。
 */
static void capture_reset_flags(void)
{
    reset_flags = 0u;

    if (RCC_GetFlagStatus(RCC_FLAG_PINRST)  != RESET) reset_flags |= RESET_CAUSE_PIN;
    if (RCC_GetFlagStatus(RCC_FLAG_PORRST)  != RESET) reset_flags |= RESET_CAUSE_POR;
    if (RCC_GetFlagStatus(RCC_FLAG_SFTRST)  != RESET) reset_flags |= RESET_CAUSE_SW;
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) reset_flags |= RESET_CAUSE_IWDG;
    if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET) reset_flags |= RESET_CAUSE_WWDG;
    if (RCC_GetFlagStatus(RCC_FLAG_LPWRRST) != RESET) reset_flags |= RESET_CAUSE_LPWR;

    RCC_ClearFlag();   /* 清除标志，避免影响下次判断 */
}

/* 输出本次启动的硬件复位标志，用于区分供电、NRST 和看门狗问题。 */
static void debug_reset_flags(void)
{
    dbg_s("RESET pin=");     dbg_u32((reset_flags & RESET_CAUSE_PIN)  ? 1u : 0u);
    dbg_s(" por_bor=");      dbg_u32((reset_flags & RESET_CAUSE_POR)  ? 1u : 0u);
    dbg_s(" sw=");           dbg_u32((reset_flags & RESET_CAUSE_SW)   ? 1u : 0u);
    dbg_s(" iwdg=");         dbg_u32((reset_flags & RESET_CAUSE_IWDG) ? 1u : 0u);
    dbg_s(" wwdg=");         dbg_u32((reset_flags & RESET_CAUSE_WWDG) ? 1u : 0u);
    dbg_s(" lpwr=");         dbg_u32((reset_flags & RESET_CAUSE_LPWR) ? 1u : 0u);
    dbg_s("\r\n");
}

/* ============================================================
 * nRF24 调试输出
 * ============================================================ */

/* 读取关键 nRF 寄存器并输出，快速判断 SPI、频道、速率和 FIFO 状态。 */
static void dbg_nrf(void)
{
    static const uint8_t registers[] = {
        0x00u, 0x03u, 0x05u, 0x06u, 0x07u,
        0x11u, 0x17u, 0x1Cu, 0x1Du
    };
    uint8_t index;

    dbg_s("NRF REG:");
    for (index = 0u; index < sizeof(registers); ++index) {
        dbg_c(' ');
        dbg_hex(registers[index]);
        dbg_c('=');
        dbg_hex(nrf24_read_register(registers[index]));
    }
    dbg_s("\r\n");
}

/* ============================================================
 * 板级初始化
 * ============================================================ */

/* 初始化系统时钟计时、PC13 LED，并根据 STM32 UID 生成本次 boot_id。 */
static void board_init(void)
{
    GPIO_InitTypeDef g;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    g.GPIO_Pin   = GPIO_Pin_13;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &g);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);

    /* 利用 STM32 唯一 ID 与 SysTick 当前值异或，生成随机的 boot_id */
    boot_id = (uint16_t)(*(volatile uint32_t *)0x1FFFF7E8u ^
                         *(volatile uint32_t *)0x1FFFF7ECu ^
                         SysTick->VAL);
}

/* 设置业务 LED 并保存逻辑状态；Blue Pill 的 PC13 为低电平点亮。 */
static void led_set(uint8_t on)
{
    led_state = on ? 1u : 0u;
    if (led_state) {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    } else {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
}

/* ============================================================
 * 继电器控制
 * ============================================================ */

/**
 * @brief 把完整 4 位逻辑状态一次同步到 PB12~PB15。
 *
 * bit0~bit3 分别对应继电器 1~4；逻辑 1=吸合，逻辑 0=断开。
 * 根据 RELAY_ACTIVE_HIGH 决定是高电平吸合还是低电平吸合。
 */
static void relay_write_state(uint8_t next_state)
{
    uint16_t on_pins   = (uint16_t)(next_state & 0x0Fu) << 12;
    uint16_t off_pins  = (uint16_t)(RELAY_PINS & (uint16_t)~on_pins);
    uint16_t high_pins;
    uint16_t low_pins;

    if (RELAY_ACTIVE_HIGH) {
        high_pins = on_pins;
        low_pins  = off_pins;
    } else {
        high_pins = off_pins;
        low_pins  = on_pins;
    }

    /* BSRR 低 16 位置 1 输出高电平，高 16 位置 1 输出低电平，一次同步更新。 */
    GPIOB->BSRR = (uint32_t)high_pins | ((uint32_t)low_pins << 16);

    relay_state = (uint8_t)(next_state & 0x0Fu);
}

/* 把单比特通道选择转换为人类可读的 1~4 通道号。 */
static uint8_t relay_channel_from_value(uint32_t value)
{
    switch ((uint8_t)value & 0x0Fu) {
        case 0x01u: return 1u;
        case 0x02u: return 2u;
        case 0x04u: return 3u;
        case 0x08u: return 4u;
        default:    return 0u;
    }
}

/* 初始化继电器 GPIO，默认全部关闭（按 RELAY_ACTIVE_HIGH 决定电平）。 */
static void relay_init(void)
{
    GPIO_InitTypeDef g;
    uint32_t off_high = RELAY_ACTIVE_HIGH ? 0u : RELAY_PINS;
    uint32_t off_low  = RELAY_ACTIVE_HIGH ? RELAY_PINS : 0u;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* 在切换为推挽输出前先准备关闭电平，缩短上电误动作窗口。 */
    GPIOB->BSRR = off_high | (off_low << 16);

    g.GPIO_Pin   = RELAY_PINS;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &g);

    GPIOB->BSRR = off_high | (off_low << 16);
    relay_state = 0u;
}

/* nRF SPI 自检失败后的致命状态：打印错误并快速闪烁，不进入业务循环。 */
static void fatal_radio(void)
{
    dbg_s("FATAL: NRF SPI\r\n");
    for (;;) {
        led_set((millis() / 100u) & 1u);
    }
}

/* ============================================================
 * 传感器状态调试输出
 * ============================================================ */

/* 输出 AHT20 在线状态、BMP280 地址和当前采样周期。 */
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

/* ============================================================
 * 响应帧构建
 * ============================================================ */

/**
 * @brief 根据 POLL 或 COMMAND 生成 32 字节应用响应。
 *
 * POLL 返回最近完整传感器快照；COMMAND 校验参数、执行并缓存结果。
 *
 * @param r   收到的帧头
 * @param p   收到的载荷指针
 * @param out 输出缓冲区（至少 32 字节）
 * @return 成功为 32，不支持的请求为 0
 */
static uint8_t build_response(const AppHeader *r, const uint8_t *p, uint8_t *out)
{
    /* ---------- POLL：返回遥测快照 ---------- */
    if (r->type == APP_MSG_POLL && r->payload_len == 0u) {
        SensorSnapshot s;
        AppTelemetry   t;
        uint32_t       age;

        sensors_get_snapshot(&s);
        age = millis() - s.sampled_at_ms;

        t.request_seq         = r->seq;
        t.temperature_centi_c = s.temperature_centi_c;
        t.humidity_centi_rh   = s.humidity_centi_rh;
        t.pressure_pa         = s.pressure_pa;
        t.sample_age_ms       = (uint16_t)(age > 65535u ? 65535u : age);
        t.sensor_status       = s.status;

        if (master_was_lost) t.sensor_status |= SENSOR_MASTER_LOST;
        t.led_state = led_state;

        return app_build_frame(out, APP_MSG_TELEMETRY, APP_NODE_ID, APP_MASTER_ID,
                               boot_id, ++tx_seq, &t, sizeof(t));
    }

    /* ---------- COMMAND：执行命令并返回 ACK ---------- */
    if (r->type == APP_MSG_COMMAND && r->payload_len == sizeof(AppCommand)) {
        AppCommand    c;
        AppCommandAck a;
        memcpy(&c, p, sizeof(c));

        /* 只有 id + code + value 完全相同才去重；
         * 允许调试时复用 id 测试不同 value。 */
        if (last_command_valid &&
            c.command_id == last_command_id &&
            c.command    == last_command_code &&
            c.value      == last_command_value) {
            dbg_s("CMD DUPLICATE id=");
            dbg_u32(c.command_id);
            dbg_s("\r\n");
            return app_build_frame(out, APP_MSG_COMMAND_ACK, APP_NODE_ID,
                                   APP_MASTER_ID, boot_id, ++tx_seq,
                                   &last_command_ack, sizeof(last_command_ack));
        }

        a.command_id = c.command_id;
        a.command    = c.command;
        a.result     = APP_RESULT_OK;
        a.value      = c.value;

        switch (c.command) {
            case APP_CMD_SET_LED:
                if (c.value > 1u) {
                    a.result = APP_RESULT_INVALID_PARAM;
                } else {
                    led_set((uint8_t)c.value);
                    a.value = led_state;
                    dbg_s("LED SET state=");     dbg_u32(led_state);
                    dbg_s(" gpio=");              dbg_u32(GPIOC->ODR & GPIO_Pin_13 ? 1u : 0u);
                    dbg_s("\r\n");
                }
                break;

            case APP_CMD_SET_RELAY: {
                uint8_t next_state;
                uint8_t channel = relay_channel_from_value(c.value);

                /* 单路命令：channel 必须能识别，且 value 通过应用层校验 */
                if (!channel ||
                    !app_relay_apply(relay_state, c.value, &next_state)) {
                    a.result = APP_RESULT_INVALID_PARAM;
                    break;
                }

                relay_write_state(next_state);
                a.value = relay_state;

                dbg_s("RELAY SET channel=");   dbg_u32(channel);
                dbg_s(" state_mask=");         dbg_u32(relay_state);
                dbg_s(" gpio=0x");             dbg_hex16((uint16_t)(GPIOB->ODR & 0xF000u));
                dbg_s("\r\n");
                break;
            }

            case APP_CMD_GET_RELAY_STATE:
                a.value = relay_state;
                dbg_s("RELAY GET state=");
                dbg_u32(relay_state);
                dbg_s("\r\n");
                break;

            case APP_CMD_SET_SAMPLE_PERIOD:
                if (c.value < APP_MIN_SAMPLE_MS || c.value > APP_MAX_SAMPLE_MS) {
                    a.result = APP_RESULT_INVALID_PARAM;
                } else {
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

        /* 缓存命令和 ACK，用于后续去重 */
        last_command_id    = c.command_id;
        last_command_ack   = a;
        last_command_valid = 1u;
        last_command_code  = c.command;
        last_command_value = c.value;

        dbg_s("CMD id=");     dbg_u32(c.command_id);
        dbg_s(" code=");      dbg_u32(c.command);
        dbg_s(" result=");    dbg_u32(a.result);
        dbg_s("\r\n");

        return app_build_frame(out, APP_MSG_COMMAND_ACK, APP_NODE_ID,
                               APP_MASTER_ID, boot_id, ++tx_seq,
                               &a, sizeof(a));
    }

    return 0u;
}

/* ============================================================
 * 无线接收与响应任务
 * ============================================================ */

/**
 * @brief 从机无线业务状态机，由主循环高频调用。
 *
 * 流程：取包 → 应用校验/身份检查 → 去重 → 构造响应 → PTX 发送 → 恢复 PRX。
 */
static void radio_task(void)
{
    uint8_t          rx[32], n, response;
    const AppHeader *h;
    const uint8_t   *p;

    /* 1. 检查是否有新数据到达 */
    if (!nrf24_receive(rx, &n)) return;

    /* 2. 校验帧格式和 CRC */
    if (!app_validate_frame(rx, n, &h, &p)) {
        dbg_s("RX DROP: CRC/FORMAT\r\n");
        return;
    }

    /* 3. 身份检查：只接受来自主机且发往本节点 ID 的帧 */
    if (h->src_id != APP_MASTER_ID || h->dst_id != APP_NODE_ID) {
        dbg_s("RX DROP: ID\r\n");
        return;
    }

    /* 4. 收到命令/轮询时的接收日志 */
    if (h->type == APP_MSG_COMMAND && h->payload_len == sizeof(AppCommand)) {
        AppCommand received;
        memcpy(&received, p, sizeof(received));
        dbg_s("RX CMD id=");    dbg_u32(received.command_id);
        dbg_s(" code=");        dbg_u32(received.command);
        dbg_s(" value=");       dbg_u32(received.value);
        dbg_s(" seq=");         dbg_u32(h->seq);
        dbg_s("\r\n");
    } else if (h->type == APP_MSG_POLL && h->payload_len == 0u) {
        dbg_s("RX POLL seq=");
        dbg_u32(h->seq);
        dbg_s("\r\n");
    }

    /* 5. 判断主机是否曾丢失 */
    master_was_lost = ((uint32_t)(millis() - last_master_ms) > APP_MASTER_LOST_MS) ? 1u : 0u;

    /* 6. 请求去重：同 boot_id + seq 的重发直接复用缓存响应 */
    if (cached_response_len &&
        h->boot_id == cached_master_boot &&
        h->seq     == cached_request_seq) {
        response = cached_response_len;
        dbg_s("RX DUPLICATE\r\n");
    } else {
        response = build_response(h, p, cached_response);
        if (!response) return;

        cached_response_len = response;
        cached_master_boot  = h->boot_id;
        cached_request_seq  = h->seq;
    }

    /*
     * SETUP_RETR=0x35 表示约 1.5ms 重发间隔、最多 3 次重发。
     * 保守等待 20ms 覆盖 1Mbps 下 32 字节请求的硬件重发和模式切换窗口，
     * 避免主从机同时处于 PTX。
     */
    {
        uint32_t started = millis();
        while ((uint32_t)(millis() - started) < APP_RESPONSE_TURNAROUND_MS) {}
    }

    /* 7. 发送响应帧（最多重试 RESPONSE_TX_RETRIES 次） */
    {
        uint8_t attempt;
        uint8_t sent = 0u;

        dbg_s("TX BEGIN response=");
        dbg_s(h->type == APP_MSG_COMMAND ? "CMD_ACK" : "TELEMETRY");
        dbg_s(" req_seq=");    dbg_u32(h->seq);
        dbg_s("\r\n");

        for (attempt = 1u; attempt <= RESPONSE_TX_RETRIES; ++attempt) {
            if (nrf24_send(MASTER_ADDRESS, cached_response, response, 30u)) {
                sent = 1u;
                break;
            }
            if (attempt < RESPONSE_TX_RETRIES) {
                dbg_s("TX RETRY attempt=");
                dbg_u32(attempt);
                dbg_s("\r\n");
            }
        }

        if (!sent) {
            dbg_s("TX FAIL response=");
            dbg_s(h->type == APP_MSG_COMMAND ? "CMD_ACK" : "TELEMETRY");
            dbg_s(" status=0x");    dbg_hex(nrf24_read_register(0x07u));
            dbg_s(" observe=0x");   dbg_hex(nrf24_read_register(0x08u));
            dbg_s(" fifo=0x");      dbg_hex(nrf24_read_register(0x17u));
            dbg_s("\r\n");
        } else {
            dbg_s("TX OK response=");
            dbg_s(h->type == APP_MSG_COMMAND ? "CMD_ACK" : "TELEMETRY");
            dbg_s(" attempt=");
            dbg_u32(attempt);
            dbg_s("\r\n");
        }
    }

    /* 8. 更新最后收到主机帧的时间，并恢复接收模式 */
    last_master_ms = millis();
    nrf24_start_listening(SLAVE_ADDRESS);
}

/* ============================================================
 * 程序入口
 * ============================================================ */

/**
 * @brief 程序入口：初始化调试、无线、传感器后永久运行采集和无线两个非阻塞任务。
 *
 * 启动顺序很重要：
 *   1) 先采集复位来源（必须在 RCC_ClearFlag 之前）；
 *   2) 初始化板级、调试串口、继电器；
 *   3) 初始化 nRF24 并自检，失败进入死循环；
 *   4) 初始化传感器并立即触发首次采样；
 *   5) 进入主循环，交替执行 sensors_task 和 radio_task。
 */
int main(void)
{
    capture_reset_flags();       /* 必须在其他初始化之前 */
    board_init();
    debug_uart_init();
    relay_init();

    /* 打印启动横幅、固件版本、复位来源、继电器配置 */
    dbg_s("\r\nSTM32 NODE BOOT id=");
    dbg_u32(APP_NODE_ID);
    dbg_s(" fw=");
    dbg_s(APP_FW_VERSION);
    dbg_s("\r\n");

    debug_reset_flags();

    dbg_s("RELAY CFG active_low=");
    dbg_u32(RELAY_ACTIVE_HIGH ? 0u : 1u);
    dbg_s(" active_high=");
    dbg_u32(RELAY_ACTIVE_HIGH);
    dbg_s(" state=0 gpio=0x");
    dbg_hex16((uint16_t)(GPIOB->ODR & 0xF000u));
    dbg_s("\r\n");

    led_set(0u);

    /* nRF24 初始化与自检 */
    nrf24_init();
    dbg_nrf();
    if (!nrf24_is_present()) {
        fatal_radio();
    }
    dbg_s("NRF PASS RF_SETUP=0x");
    dbg_hex(nrf24_read_register(0x06u));
    dbg_s("\r\n");

    /* 传感器初始化并立即触发首次采样 */
    sensors_init();
    debug_sensor_status();
    sensors_force_sample();

    last_master_ms = millis();
    nrf24_start_listening(SLAVE_ADDRESS);

    /* 主循环：非阻塞处理传感器采集和无线通信 */
    for (;;) {
        uint32_t now = millis();
        sensors_task(now);
        radio_task();
    }
}