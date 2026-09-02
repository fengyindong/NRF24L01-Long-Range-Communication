#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "app_protocol.h"
#include "nrf24.h"
#include "config.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ------------------------- 枚举类型定义 ------------------------- */
typedef enum {
    NODE_UNKNOWN,
    NODE_ONLINE,
    NODE_DEGRADED,
    NODE_OFFLINE,
    NODE_RECOVERING
} NodeState;

typedef enum {
    RF_IDLE,
    RF_SEND,
    RF_WAIT
} RadioState;

typedef enum {
    CLM_AT,             /* 测试通信 */
    CLM_VERBOSE,        /* 开启详细错误 */
    CLM_SIM_QUERY,      /* 查询SIM卡 */
    CLM_SIGNAL_QUERY,   /* 查询信号 */
    CLM_REG_QUERY,      /* 查询网络注册 */
    CLM_ATTACH_QUERY,   /* 查询附着状态 */
    CLM_PDP_QUERY,      /* 查询PDP上下文 */
    CLM_PDP_CONFIG,     /* 配置APN */
    CLM_ATTACH,         /* 请求附着 */
    CLM_ACTIVATE,       /* 激活PDP */
    CLM_MQTT_CONFIG,    /* 配置MQTT参数 */
    CLM_CONNECT,        /* 连接MQTT */
    CLM_SUBSCRIBE,      /* 订阅主题 */
    CLM_READY,          /* 就绪，可收发数据 */
    CLM_BACKOFF         /* 退避重连 */
} ClmState;

typedef enum {
    CLM_WAIT_NONE,
    CLM_WAIT_OK,            /* 等待普通OK */
    CLM_WAIT_CONNECT_URC,   /* 等待连接成功URC */
    CLM_WAIT_SUB_URC,       /* 等待订阅成功URC */
    CLM_WAIT_PUB_URC,       /* 等待发布成功URC */
    CLM_WAIT_PUB_PROMPT,    /* 等待PUBEX的>提示符 */
    CLM_WAIT_STATE_URC      /* 等待状态查询URC */
} ClmWait;

/* ------------------------- 结构体定义 ------------------------- */
#pragma pack(push, 1)
typedef struct {
    uint16_t request_seq;
    int16_t temperature_centi_c;
    uint16_t humidity_centi_rh;
    uint32_t pressure_pa;
    uint16_t sample_age_ms;
    uint8_t sensor_status;
    uint8_t led_state;
} Telemetry;

typedef struct {
    uint32_t command_id;
    uint8_t command;
    uint8_t reserved;
    uint32_t value;
} Command;

typedef struct {
    uint32_t command_id;
    uint8_t command;
    uint8_t result;
    uint32_t value;
} CommandAck;
#pragma pack(pop)

typedef struct {
    uint8_t id;
    NodeState state;
    uint8_t failures, successes;
    uint32_t last_seen, next_poll;
    uint16_t boot, last_seq;
    uint8_t has_seq;
    Telemetry data;
} Node;

typedef struct {
    uint8_t node, code;
    uint32_t id, value;
} Cmd;

typedef struct {
    char topic[96];
    char payload[MQTT_PAYLOAD_MAX];
    uint8_t retained;
} Pub;

/* ------------------------- 全局变量定义 ------------------------- */
static volatile uint32_t g_ms;

static Node nodes[NODE_COUNT];
static Cmd cmds[8];
static uint8_t ch, ct, cc;

static Pub pubs[MQTT_QUEUE_SIZE];
static uint8_t ph, pt, pc;

static uint8_t rf_frame[32], rf_expected, rf_node, rf_attempts, rf_cmd;
static uint16_t rf_seq, seq_next, master_boot;
static uint32_t rf_deadline, rf_retry, rf_cmd_id, rf_cmd_value;
static uint8_t rf_command_active;        /* 标记当前是否有正在发送的命令 */
static uint8_t rf_cmd_slot;              /* 当前命令在环形队列中的槽位 */

static RadioState rf_state = RF_IDLE;
static ClmState clm_state = CLM_AT;
static ClmWait clm_wait_kind = CLM_WAIT_NONE;

static uint8_t clm_connected, clm_cmd_index, clm_waiting;
static uint8_t mqtt_pub_pending, mqtt_subscribed;
static uint8_t clm_sim_ready, clm_registered, clm_attached;
static uint8_t clm_pdp_active, clm_backoff_count;

static uint32_t clm_next, clm_deadline, clm_diag_next, mqtt_pub_deadline;
static uint32_t clm_rx_bytes, clm_uart_errors;

/* 定义UART2接收环形缓冲区，利用中断保护数据不丢失 */
#define CLM_RX_RING_SIZE 512u
static volatile uint8_t clm_rx_ring[CLM_RX_RING_SIZE];
static volatile uint16_t clm_rx_head, clm_rx_tail;

static char clm_line[760];
static uint16_t clm_len;
static char pc_line[96];
static uint8_t pc_len;
static char clm_cmd_buffer[760]; /* AT命令缓冲；消息正文通过PUBEX提示符单独发送 */

static const uint8_t MASTER_ADDR[5] = {0xD2, 0xD2, 0xD2, 0xD2, 0xA0};

/* ------------------------- 前置函数声明 ------------------------- */
static void clm_parse_pub(char *s);
static void publish_gateway_state(const char *state);
static void pub_add(const char *t, const char *p, uint8_t retained);

/* 1 ms 系统节拍。 */
void SysTick_Handler(void) {
    ++g_ms;
}

uint32_t millis(void) {
    return g_ms;
}

/* 向指定串口发送一个字符。 */
static void putc_u(USART_TypeDef *u, char c) {
    while (USART_GetFlagStatus(u, USART_FLAG_TXE) == RESET) {}
    USART_SendData(u, (uint8_t)c);
}

/* 向调试串口输出字符串。 */
static void dbg(const char *s) {
    while (*s) putc_u(USART1, *s++);
}

/* 输出无符号整数，避免引入重量级 printf。 */
static void dbg_u32(uint32_t v) {
    char b[11];
    uint8_t n = 0;

    if (!v) {
        putc_u(USART1, '0');
        return;
    }

    while (v) {
        b[n++] = (char)('0' + v % 10u);
        v /= 10u;
    }

    while (n) putc_u(USART1, b[--n]);
}

/* 输出十六进制字节。 */
static void dbg_hex(uint8_t v) {
    const char *h = "0123456789ABCDEF";
    putc_u(USART1, h[v >> 4]);
    putc_u(USART1, h[v & 15]);
}

/* 读取关键 nRF24 寄存器，诊断 SPI 是否返回 0xFF/0x00 或异常值。 */
static void nrf_dump(void) {
    static const uint8_t r[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x11, 0x17};
    uint8_t i;

    dbg("NRF REG");
    for (i = 0; i < sizeof(r); ++i) {
        dbg(" ");
        dbg_hex(r[i]);
        dbg("=");
        dbg_hex(nrf24_read_register(r[i]));
    }
    dbg("\r\n");
}

/* 发送 CLR970 AT 命令并自动补充 CRLF。 */
static void clm_send(const char *s) {
    if (DEBUG_VERBOSE || strstr(s, "IMQTTCONN") || strstr(s, "IMQTTSUB") || strstr(s, "IMQTTPUB")) {
        dbg("CLM >> [");
        dbg(s);
        dbg("]\r\n");
    }
    while (*s) putc_u(USART2, *s++);
    putc_u(USART2, '\r');
    putc_u(USART2, '\n');
}

/* 获取有效APN；编译配置为空时使用明确的物联网卡APN，杜绝发送空APN。 */
static const char *effective_apn(void) {
    static const char default_apn[] = "CMIOTGZSW.GD.MNC008.MCC460.GPRS";
    if (CLM_APN[0]) return CLM_APN;
    if (CLM_APN_FALLBACK[0]) return CLM_APN_FALLBACK;
    return default_apn;
}

/* Client ID 不允许为空；即使Keil误引用了空配置，也使用网关专用兜底ID。 */
static const char *effective_client_id(void) {
    return MQTT_CLIENT_ID[0] ? MQTT_CLIENT_ID : "stm32_gateway01_clm970_fallback";
}

/* 发送命令并记录所等待的应答类型，避免在异步URC到达前重复发送。 */
static void clm_issue(const char *s, ClmWait wait, uint32_t timeout_ms) {
    clm_send(s);
    clm_wait_kind = wait;
    clm_waiting = 1;
    clm_deadline = millis() + timeout_ms;
}

/* 指数退避进入重连；MQTT队列不清空，以便网络恢复后继续发送。 */
static void clm_enter_backoff(const char *reason) {
    uint32_t delay_ms = MQTT_RECONNECT_MS;
    uint8_t i;

    for (i = 0; i < clm_backoff_count && delay_ms < CLM_BACKOFF_MAX_MS / 2u; i++) delay_ms *= 2u;
    if (delay_ms > CLM_BACKOFF_MAX_MS) delay_ms = CLM_BACKOFF_MAX_MS;
    if (clm_backoff_count < 6u) clm_backoff_count++;

    clm_connected = 0;
    mqtt_subscribed = 0;
    mqtt_pub_pending = 0;
    clm_waiting = 0;
    clm_wait_kind = CLM_WAIT_NONE;
    clm_state = CLM_BACKOFF;
    clm_next = millis() + delay_ms;

    publish_gateway_state("offline");
    dbg("CLM BACKOFF reason=");
    dbg(reason);
    dbg(" delay_ms=");
    dbg_u32(delay_ms);
    dbg("\r\n");
}

/* USART1/USART2 初始化。 USART2 使用RX中断和环形缓冲，避免主循环处理nRF时丢失CLR970字节。 */
static void uart_init(void) {
    GPIO_InitTypeDef g;
    USART_InitTypeDef u;
    NVIC_InitTypeDef n;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode = GPIO_Mode_AF_PP;
    g.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_2;
    GPIO_Init(GPIOA, &g);

    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    g.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &g);

    USART_StructInit(&u);
    u.USART_BaudRate = DEBUG_BAUD;
    u.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &u);

    u.USART_BaudRate = CLM_BAUD;
    USART_Init(USART2, &u);

    USART_Cmd(USART1, ENABLE);
    USART_Cmd(USART2, ENABLE);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    n.NVIC_IRQChannel = USART2_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 2;
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
}

/* USART2 RX中断：按STM32F1规定读SR后读DR，清除并记录错误，不丢弃正常字节。 */
void USART2_IRQHandler(void) {
    uint16_t sr = USART2->SR;
    uint8_t c;
    uint16_t next;

    if (sr & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE)) {
        c = (uint8_t)(USART2->DR & 0x00FFu);
        if (sr & (USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE)) ++clm_uart_errors;
        if (sr & USART_FLAG_RXNE) {
            next = (uint16_t)((clm_rx_head + 1u) % CLM_RX_RING_SIZE);
            if (next != clm_rx_tail) {
                clm_rx_ring[clm_rx_head] = c;
                clm_rx_head = next;
                ++clm_rx_bytes;
            }
        }
    }
}

/* 非阻塞读取串口字符。
 * 重要：STM32F1 的 USART 状态位必须按“读 SR 后读 DR”清除，
 * 不能在检测 ORE/FE/NE/PE 时先另读一次 DR，否则会把正常字节丢掉（例如 OK 变成 K）。 */
static uint8_t readc(USART_TypeDef *u, char *c) {
    uint16_t tail;

    if (u == USART2) {
        if (clm_rx_tail == clm_rx_head) return 0u;
        tail = clm_rx_tail;
        *c = (char)clm_rx_ring[tail];
        clm_rx_tail = (uint16_t)((tail + 1u) % CLM_RX_RING_SIZE);
        return 1u;
    }

    uint16_t sr;
    sr = u->SR;

    if ((sr & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE)) == 0u) return 0u;
    
    /* 只读一次 DR；即使同时有错误标志，也保留该字节交给上层。 */
    *c = (char)(u->DR & 0x00FFu);
    return 1u;
}

/* 处理普通OK；连接/订阅/发布的OK只表示命令已接受，还要继续等待URC结果。 */
static void clm_handle_ok(void) {
    if (clm_wait_kind != CLM_WAIT_OK) return;

    clm_waiting = 0;
    clm_wait_kind = CLM_WAIT_NONE;
    clm_next = millis() + 200u;

    switch (clm_state) {
        case CLM_AT:
            clm_state = CLM_VERBOSE;
            break;

        case CLM_VERBOSE:
            clm_state = CLM_SIM_QUERY;
            break;

        case CLM_SIM_QUERY:
            if (clm_sim_ready) clm_state = CLM_SIGNAL_QUERY;
            else clm_enter_backoff("SIM_NOT_READY");
            break;

        case CLM_SIGNAL_QUERY:
            clm_state = CLM_REG_QUERY;
            break;

        case CLM_REG_QUERY:
            if (clm_registered) clm_state = CLM_ATTACH_QUERY;
            else clm_enter_backoff("NOT_REGISTERED");
            break;

        case CLM_ATTACH_QUERY:
            if (clm_attached) clm_state = CLM_PDP_QUERY;
            else clm_state = CLM_PDP_CONFIG;
            break;

        case CLM_PDP_QUERY:
            if (clm_pdp_active) {
                clm_state = CLM_MQTT_CONFIG;
                clm_cmd_index = 0;
            } else {
                clm_state = CLM_PDP_CONFIG;
            }
            break;

        case CLM_PDP_CONFIG:
            clm_state = CLM_ATTACH;
            break;

        case CLM_ATTACH:
            clm_state = CLM_ACTIVATE;
            break;

        case CLM_ACTIVATE:
            clm_state = CLM_MQTT_CONFIG;
            clm_cmd_index = 0;
            break;

        case CLM_MQTT_CONFIG:
            clm_cmd_index++;
            if (clm_cmd_index >= 4u) clm_state = CLM_CONNECT;
            break;

        default:
            break;
    }
}

/* 按行解析 CLR970 URC/响应。 */
static void clm_handle_line(char *s) {
    int v;
    char *comma;

    if (DEBUG_VERBOSE || strstr(s, "+IMQTT") || strstr(s, "ERROR") || !strcmp(s, "RDY") || strstr(s, "ATREADY")) {
        dbg("CLM << [");
        dbg(s);
        dbg("]\r\n");
    }

    /* RDY 或 *ATREADY 表示模块发生了重启，原MQTT会话必然已失效。 */
    if (!strcmp(s, "RDY") || strstr(s, "ATREADY")) {
        clm_connected = 0;
        mqtt_subscribed = 0;
        mqtt_pub_pending = 0;
        clm_waiting = 0;
        clm_wait_kind = CLM_WAIT_NONE;
        clm_state = CLM_AT;
        clm_next = millis() + CLM_BOOT_DELAY_MS;
        dbg("CLM REBOOT DETECTED -> REINIT\r\n");
        return;
    }

    if (!strcmp(s, "OK")) {
        clm_handle_ok();
        return;
    }

    if (strstr(s, "ERROR")) {
        clm_enter_backoff("AT_ERROR");
        return;
    }

    if (!strncmp(s, "+CPIN:", 6)) {
        clm_sim_ready = (strstr(s, "READY") != 0);
        return;
    }

    if (!strncmp(s, "+CEREG:", 7)) {
        comma = strrchr(s, ',');
        v = atoi(comma ? comma + 1 : s + 7);
        clm_registered = (v == 1 || v == 5);
        dbg("NET registered=");
        dbg_u32(clm_registered);
        dbg("\r\n");
        return;
    }

    if (!strncmp(s, "+CGATT:", 7)) {
        clm_attached = (atoi(s + 7) == 1);
        dbg("NET attached=");
        dbg_u32(clm_attached);
        dbg("\r\n");
        return;
    }

    if (!strncmp(s, "+CGACT:", 7)) {
        char *p = s + 7;
        while (*p == ' ') p++;
        if (atoi(p) == 1 && (comma = strchr(p, ',')) != 0) clm_pdp_active = (atoi(comma + 1) == 1);
        dbg("NET pdp_active=");
        dbg_u32(clm_pdp_active);
        dbg("\r\n");
        return;
    }

    if (!strncmp(s, "+IMQTTSTATE:", 12)) {
        v = atoi(s + 12);
        clm_waiting = 0;
        clm_wait_kind = CLM_WAIT_NONE;
        if (v == 1 || v == 2) {
            clm_connected = 1;
            clm_backoff_count = 0;
            clm_state = mqtt_subscribed ? CLM_READY : CLM_SUBSCRIBE;
            clm_next = millis() + 200u;
            clm_diag_next = millis() + MQTT_STATE_CHECK_MS;
            dbg("MQTT ONLINE state=");
            dbg_u32((uint32_t)v);
            dbg("\r\n");
        } else {
            clm_enter_backoff("MQTT_STATE_0");
        }
        return;
    }

    if (!strncmp(s, "+IMQTTCONN:", 11)) {
        v = atoi(s + 11);
        clm_waiting = 0;
        clm_wait_kind = CLM_WAIT_NONE;
        if (v == 0 || v == 1) {
            clm_connected = 1;
            clm_backoff_count = 0;
            clm_state = CLM_SUBSCRIBE;
            clm_next = millis() + 200u;
            clm_diag_next = millis() + MQTT_STATE_CHECK_MS;
            dbg("MQTT CONNECTED result=");
            dbg_u32((uint32_t)v);
            dbg("\r\n");
        } else {
            clm_enter_backoff("MQTT_CONNECT_FAIL");
        }
        return;
    }

    if (!strncmp(s, "+IMQTTSUB:", 10)) {
        mqtt_subscribed = 1;
        clm_waiting = 0;
        clm_wait_kind = CLM_WAIT_NONE;
        clm_state = CLM_READY;
        clm_next = millis() + 100u;
        publish_gateway_state("online");
        dbg("MQTT SUBSCRIBED\r\n");
        return;
    }

    if (!strncmp(s, "+IMQTTPUB:", 10) || !strncmp(s, "+IMQTTPUBEX:", 12)) {
        comma = strrchr(s, ',');
        clm_waiting = 0;
        clm_wait_kind = CLM_WAIT_NONE;
        if (comma && atoi(comma + 1) == 0 && mqtt_pub_pending) {
            ph = (ph + 1u) % MQTT_QUEUE_SIZE;
            --pc;
            mqtt_pub_pending = 0;
            clm_next = millis() + 50u;
            dbg("MQTT PUB ACK queue=");
            dbg_u32(pc);
            dbg("\r\n");
        } else {
            mqtt_pub_pending = 0;
            clm_enter_backoff("MQTT_PUB_FAIL");
        }
        return;
    }

    if (!strncmp(s, "+IMQTTRCVPUB:", 13)) clm_parse_pub(s);
}

/* IMQTTPUBEX 返回“>”提示后，发送指定长度的原始消息，不再附加CRLF。 */
static void clm_handle_prompt(void) {
    uint16_t i;
    size_t n;

    if (clm_wait_kind != CLM_WAIT_PUB_PROMPT || !mqtt_pub_pending || !pc) return;

    n = strlen(pubs[ph].payload);
    for (i = 0; i < n; i++) putc_u(USART2, pubs[ph].payload[i]);

    clm_wait_kind = CLM_WAIT_PUB_URC;
    clm_deadline = millis() + MQTT_PUBLISH_TIMEOUT_MS;
    dbg("MQTT PUB DATA sent len=");
    dbg_u32((uint32_t)n);
    dbg("\r\n");
}

/* 接收 CLR970 数据，按 CR/LF 分行。 */
static void clm_rx_task(void) {
    char c;

    while (readc(USART2, &c)) {
        if (c == '>') {
            clm_handle_prompt();
            continue;
        }
        if (c == '\r') continue;

        if (c == '\n') {
            if (clm_len) {
                clm_line[clm_len] = 0;
                clm_handle_line(clm_line);
                clm_len = 0;
            }
        } else if (clm_len < sizeof(clm_line) - 1) {
            clm_line[clm_len++] = c;
        }
    }
}

/* 将 UTF-8/ASCII JSON 数字字段解析为无符号整数。 */
static uint32_t json_num(const char *s, const char *k) {
    char key[40];
    char *p;
    snprintf(key, sizeof(key), "\"%s\"", k);
    p = strstr((char *)s, key);

    if (!p || (p = strchr(p, ':')) == 0) return 0;

    return (uint32_t)strtoul(p + 1, 0, 10);
}

/* 入队一条 MQTT 发布；离线时同主题遥测只保留最新值，但绝不覆盖正在等待ACK的队首。 */
static void pub_add(const char *t, const char *p, uint8_t retained) {
    uint8_t i, q, start = mqtt_pub_pending ? 1u : 0u;

    for (q = start, i = (uint8_t)((ph + start) % MQTT_QUEUE_SIZE);
         q < pc;
         q++, i = (uint8_t)((i + 1u) % MQTT_QUEUE_SIZE)) {
        if (!strcmp(pubs[i].topic, t) && (!retained || pubs[i].retained)) {
            strncpy(pubs[i].payload, p, MQTT_PAYLOAD_MAX - 1u);
            pubs[i].payload[MQTT_PAYLOAD_MAX - 1u] = 0;
            pubs[i].retained = retained;
            return;
        }
    }

    if (pc >= MQTT_QUEUE_SIZE) {
        dbg("MQTT PUB DROP full topic=");
        dbg(t);
        dbg("\r\n");
        return;
    }

    strncpy(pubs[pt].topic, t, sizeof(pubs[pt].topic) - 1u);
    strncpy(pubs[pt].payload, p, MQTT_PAYLOAD_MAX - 1u);
    pubs[pt].topic[sizeof(pubs[pt].topic) - 1u] = 0;
    pubs[pt].payload[MQTT_PAYLOAD_MAX - 1u] = 0;
    pubs[pt].retained = retained;
    pt = (uint8_t)((pt + 1u) % MQTT_QUEUE_SIZE);
    ++pc;
}

/* 发布网关自身的在线/离线状态；使用 retained 便于 MQTTX 重新订阅时立即看到状态。 */
static void publish_gateway_state(const char *state) {
    char topic[96];
    char json[160];

    snprintf(topic, sizeof(topic), "%s/gateway/state", MASTER_MQTT_ROOT);
    snprintf(json, sizeof(json), "{\"gateway_id\":\"%s\",\"state\":\"%s\",\"uptime_s\":%lu}",
             GATEWAY_ID, state, (unsigned long)(millis() / 1000u));
    pub_add(topic, json, 1u);
}

/* 处理 MQTT 下行命令并加入无线命令队列。 */
static void clm_message(const char *topic, const char *msg) {
    uint8_t node = (uint8_t)json_num(msg, "node_id");
    uint8_t code = (uint8_t)json_num(msg, "code");
    uint32_t id = json_num(msg, "command_id");
    uint32_t value = json_num(msg, "value");
    uint8_t i, known = 0;

    if (!strstr(topic, "/command/set")) return;

    for (i = 0; i < NODE_COUNT; i++) {
        if (NODE_IDS[i] == node) known = 1u;
    }

    if (!known || !node || !code || !id || cc >= 8) {
        dbg("MQTT CMD DROP node=");
        dbg_u32(node);
        dbg(" id=");
        dbg_u32(id);
        dbg("\r\n");
        return;
    }

    cmds[ct] = (Cmd){node, code, id, value};
    ct = (uint8_t)((ct + 1u) % 8u);
    ++cc;

    /* 命令到达即唤醒目标节点，不再等待正常遥测轮询周期。 */
    for (i = 0; i < NODE_COUNT; i++) {
        if (nodes[i].id == node) nodes[i].next_poll = millis();
    }

    dbg("MQTT CMD QUEUED node=");
    dbg_u32(node);
    dbg(" id=");
    dbg_u32(id);
    dbg(" code=");
    dbg_u32(code);
    dbg(" value=");
    dbg_u32(value);
    dbg("\r\n");
}

/* 从命令环形队列删除指定槽位，保持其余待执行命令顺序。 */
static void cmd_remove_slot(uint8_t slot) {
    uint8_t i, next;
    if (!cc) return;

    i = slot;
    while (i != ct) {
        next = (uint8_t)((i + 1u) % 8u);
        if (next == ct) break;
        cmds[i] = cmds[next];
        i = next;
    }
    ct = (uint8_t)((ct + 7u) % 8u);
    --cc;
}

/* 解析 MQTT 接收 URC：+IMQTTRCVPUB:packet,topic,len,message。 */
/* 解析 +IMQTTRCVPUB:<id>,"<topic>",<len>,"<payload>"；去掉两端引号后再交给JSON命令解析。 */
static void clm_parse_pub(char *s) {
    char *p = s + 13;
    char *a, *b, *c, *topic, *msg, *e;
    uint32_t len;

    a = strchr(p, ',');
    if (!a) return;
    topic = a + 1;

    if (*topic == '"') topic++;
    b = strchr(topic, '"');
    if (!b) return;
    *b = 0;

    c = strchr(b + 1, ',');
    if (!c) return;
    len = (uint32_t)strtoul(c + 1, &e, 10);
    if (!e || *e != ',') return;

    msg = e + 1;
    if (*msg == '"') msg++;

    /* 以消息长度截断，避免末尾引号或异步URC字符进入JSON。 */
    if (len >= MQTT_PAYLOAD_MAX) len = MQTT_PAYLOAD_MAX - 1u;
    msg[len] = 0;

    clm_message(topic, msg);
    dbg("MQTT RX CMD topic=");
    dbg(topic);
    dbg(" len=");
    dbg_u32(len);
    dbg("\r\n");
}

/* CLM920 初始化、网络附着、MQTT连接及发布队列状态机。 */
static void clm_task(void) {
    uint32_t now = millis();
    char *b = clm_cmd_buffer;

    clm_rx_task();

    if (mqtt_pub_pending && (int32_t)(now - mqtt_pub_deadline) >= 0) {
        dbg("CLM PUB TIMEOUT state=");
        dbg_u32((uint32_t)clm_state);
        dbg(" baud=");
        dbg_u32(CLM_BAUD);
        dbg("\r\n");
        clm_enter_backoff("MQTT_PUB_TIMEOUT");
        return;
    }

    if (clm_waiting) {
        if ((int32_t)(now - clm_deadline) < 0) return;

        dbg("CLM RESPONSE TIMEOUT state=");
        dbg_u32((uint32_t)clm_state);
        dbg(" wait=");
        dbg_u32((uint32_t)clm_wait_kind);
        dbg(" rx_bytes=");
        dbg_u32(clm_rx_bytes);
        dbg(" uart_errors=");
        dbg_u32(clm_uart_errors);
        dbg(" baud=");
        dbg_u32(CLM_BAUD);
        dbg(" (check PA2/PA3, baud, power)\r\n");
        clm_enter_backoff("RESPONSE_TIMEOUT");
        return;
    }

    if ((int32_t)(now - clm_next) < 0) return;

    switch (clm_state) {
        case CLM_AT:
            clm_issue("AT", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_VERBOSE:
            clm_issue("AT+CMEE=2", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_SIM_QUERY:
            clm_sim_ready = 0;
            clm_issue("AT+CPIN?", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_SIGNAL_QUERY:
            clm_issue("AT+CSQ", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_REG_QUERY:
            clm_registered = 0;
            clm_issue("AT+CEREG?", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_ATTACH_QUERY:
            clm_attached = 0;
            clm_issue("AT+CGATT?", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_PDP_QUERY:
            clm_pdp_active = 0;
            clm_issue("AT+CGACT?", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_PDP_CONFIG:
            snprintf(b, sizeof(clm_cmd_buffer), "AT+CGDCONT=1,\"IP\",\"%s\"", effective_apn());
            clm_issue(b, CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_ATTACH:
            clm_issue("AT+CGATT=1", CLM_WAIT_OK, 15000u);
            break;

        case CLM_ACTIVATE:
            clm_issue("AT+CGACT=1,1", CLM_WAIT_OK, 15000u);
            break;

        case CLM_MQTT_CONFIG:
            if (clm_cmd_index == 0u) {
                snprintf(b, sizeof(clm_cmd_buffer), "AT+IMQTTMODE=2,%u,0", (unsigned)MQTT_TLS);
            } else if (clm_cmd_index == 1u) {
                snprintf(b, sizeof(clm_cmd_buffer), "AT+IMQTTPARA=TIMEOUT,2,CLEAN,1,KEEPALIVE,60,VERSION,3.1.1");
            } else if (clm_cmd_index == 2u) {
                snprintf(b, sizeof(clm_cmd_buffer), "AT+IMQTTADDR=\"%s\",%u,\"%s\"",
                         MQTT_HOST, (unsigned)MQTT_PORT, effective_client_id());
            } else {
                snprintf(b, sizeof(clm_cmd_buffer), "AT+IMQTTUSER=\"%s\",\"%s\"",
                         MQTT_USER, MQTT_PASSWORD);
            }
            clm_issue(b, CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_CONNECT:
            clm_issue("AT+IMQTTCONN", CLM_WAIT_CONNECT_URC, CLM_CONNECT_TIMEOUT_MS);
            break;

        case CLM_SUBSCRIBE:
            /* MQTT的+只匹配一级；实际命令主题为ROOT/node/<id>/command/set。 */
            snprintf(b, sizeof(clm_cmd_buffer), "AT+IMQTTSUB=%s/node/+/command/set,%u",
                     MASTER_MQTT_ROOT, (unsigned)MQTT_SUB_QOS);
            clm_issue(b, CLM_WAIT_SUB_URC, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_READY:
            if (!clm_connected) {
                clm_enter_backoff("LOCAL_DISCONNECTED");
                break;
            }

            if (pc) {
                int cmd_len;
                /* 按手册使用IMQTTPUBEX，避免JSON引号转义和超长AT行。 */
                cmd_len = snprintf(b, sizeof(clm_cmd_buffer),
                                   "AT+IMQTTPUBEX=%s,%u,0,%u",
                                   pubs[ph].topic, (unsigned)MQTT_PUB_QOS,
                                   (unsigned)strlen(pubs[ph].payload));

                if (cmd_len < 0 || cmd_len >= (int)CLM_AT_LINE_MAX) {
                    dbg("MQTT PUB DROP AT_TOO_LONG len=");
                    dbg_u32((uint32_t)cmd_len);
                    dbg(" max=");
                    dbg_u32(CLM_AT_LINE_MAX);
                    dbg("\r\n");
                    ph = (uint8_t)((ph + 1u) % MQTT_QUEUE_SIZE);
                    --pc;
                    mqtt_pub_pending = 0;
                    clm_next = now + 100u;
                    break;
                }

                clm_issue(b, CLM_WAIT_PUB_PROMPT, MQTT_PUBLISH_TIMEOUT_MS);
                mqtt_pub_pending = 1;
                mqtt_pub_deadline = clm_deadline;
                dbg("MQTT PUB queued topic=");
                dbg(pubs[ph].topic);
                dbg(" json_len=");
                dbg_u32((uint32_t)strlen(pubs[ph].payload));
                dbg(" queue=");
                dbg_u32(pc);
                dbg("\r\n");
            } else if ((int32_t)(now - clm_diag_next) >= 0) {
                clm_issue("AT+IMQTTSTATE?", CLM_WAIT_STATE_URC, CLM_COMMAND_TIMEOUT_MS);
            } else {
                clm_next = now + 200u;
            }
            break;

        case CLM_BACKOFF:
            clm_state = CLM_AT;
            clm_next = now;
            break;
    }
}

/* 根据节点ID生成与从机一致的5字节无线地址。 */
static void node_addr(uint8_t id, uint8_t a[5]) {
    a[0] = a[1] = a[2] = a[3] = 0xE7;
    a[4] = id;
}

/* 记录节点成功/失败并维护在线、降级、离线状态。 */
static void node_result(Node *n, uint8_t ok) {
    NodeState old = n->state;

    if (ok) {
        n->last_seen = millis();
        n->failures = 0;
        if (n->successes < 255) n->successes++;

        if ((n->state == NODE_UNKNOWN || n->state == NODE_OFFLINE) && n->successes >= ONLINE_RECOVERY_OK) {
            n->state = NODE_ONLINE;
        }
    } else {
        n->successes = 0;
        if (n->failures < 255) n->failures++;

        if (n->failures >= OFFLINE_FAILURES) {
            n->state = NODE_OFFLINE;
        } else if (n->failures >= DEGRADED_FAILURES) {
            n->state = NODE_DEGRADED;
        }
    }

    if (old != n->state) {
        char t[96];
        char j[160];
        const char *names[] = {"unknown", "online", "degraded", "offline", "recovering"};

        snprintf(t, sizeof(t), "environment/%s/node/%u/state", GATEWAY_ID, n->id);
        snprintf(j, sizeof(j), "{\"node_id\":%u,\"state\":\"%s\",\"failures\":%u}",
                 n->id, names[n->state], n->failures);
        pub_add(t, j, 1);
    }

    n->next_poll = millis() + (n->state == NODE_OFFLINE ? OFFLINE_PROBE_MS : NORMAL_POLL_MS);
}

/* 发布易读遥测 JSON。用整数拆分小数，不依赖 STM32 printf 的浮点支持。 */
static void publish_telemetry(Node *n, const AppHeader *h, const Telemetry *t) {
    char topic[96];
    char json[MQTT_PAYLOAD_MAX];
    int32_t tc = (int32_t)t->temperature_centi_c;
    uint32_t ta = (uint32_t)(tc < 0 ? -tc : tc);
    uint32_t hc = t->humidity_centi_rh;
    uint32_t pa = t->pressure_pa;
    uint8_t sensor_ok = ((t->sensor_status & 0x0Fu) == 0x0Fu && (t->sensor_status & 0x30u) == 0u) ? 1u : 0u;

    snprintf(topic, sizeof(topic), "%s/node/%u/telemetry", MASTER_MQTT_ROOT, n->id);

    /* 保留监控必需字段，缩短HEX后的AT命令；Pa可由hPa*100换算，不再重复上传。 */
    snprintf(json, sizeof(json),
             "{\"node_id\":%u,\"seq\":%u,\"temperature_c\":%s%lu.%02lu,\"humidity_rh\":%lu.%02lu,\"pressure_hpa\":%lu.%02lu,\"sensor_status\":%u,\"sensor_ok\":%s}",
             n->id, h->seq,
             tc < 0 ? "-" : "",
             (unsigned long)(ta / 100u), (unsigned long)(ta % 100u),
             (unsigned long)(hc / 100u), (unsigned long)(hc % 100u),
             (unsigned long)(pa / 100u), (unsigned long)(pa % 100u),
             t->sensor_status,
             sensor_ok ? "true" : "false");

    pub_add(topic, json, 0);

    if (DEBUG_VERBOSE) {
        dbg("MQTT QUEUE JSON ");
        dbg(json);
        dbg("\r\n");
    }
}

/* 发布命令回执。 */
static void publish_command_result(uint8_t node, const CommandAck *a) {
    char topic[96];
    char json[MQTT_PAYLOAD_MAX];

    snprintf(topic, sizeof(topic), "environment/%s/node/%u/command/result", GATEWAY_ID, node);
    snprintf(json, sizeof(json),
             "{\"node_id\":%u,\"command_id\":%lu,\"code\":%u,\"result\":%u,\"value\":%lu}",
             node, (unsigned long)a->command_id,
             a->command, a->result, (unsigned long)a->value);
    pub_add(topic, json, 0);
}

/* 发送一轮POLL或命令，等待从机应用层响应并进行三次重试。 */
static void radio_task(void) {
    static uint8_t index;
    static uint16_t seq;
    static uint32_t next_try;
    static Cmd cmd;
    static uint8_t frame[32], rx[32], len;
    const AppHeader *h;
    const uint8_t *p;
    uint32_t now = millis();
    Node *np;

    if (rf_state == RF_IDLE) {
        uint8_t selected = 0u;

        if (cc) {
            uint8_t q;
            for (q = 0; q < cc && !selected; q++) {
                uint8_t slot = (uint8_t)((ch + q) % 8u);
                for (index = 0; index < NODE_COUNT; index++) {
                    if (nodes[index].id == cmds[slot].node) {
                        rf_node = nodes[index].id;
                        cmd = cmds[slot];
                        rf_cmd_slot = slot;
                        selected = 1u;
                        break;
                    }
                }
            }
        } else {
            for (index = 0; index < NODE_COUNT; index++) {
                if ((int32_t)(now - nodes[index].next_poll) >= 0) {
                    rf_node = nodes[index].id;
                    selected = 1u;
                    break;
                }
            }
        }

        if (!selected) return;

        rf_seq = ++seq;

        if (cc && cmd.id && cmd.node == rf_node) {
            rf_command_active = 1u;
            rf_cmd_id = cmd.id;
            Command c = {cmd.id, cmd.code, 0, cmd.value};
            app_build_frame(frame, APP_MSG_COMMAND, 0, rf_node, master_boot, rf_seq, &c, sizeof(c));
            rf_expected = APP_MSG_COMMAND_ACK;
            dbg("RF CMD SEND node=");
            dbg_u32(rf_node);
            dbg(" id=");
            dbg_u32(cmd.id);
            dbg(" code=");
            dbg_u32(cmd.code);
            dbg(" value=");
            dbg_u32(cmd.value);
            dbg("\r\n");
        } else {
            rf_command_active = 0u;
            rf_cmd_id = 0u;
            cmd.id = 0u;
            app_build_frame(frame, APP_MSG_POLL, 0, rf_node, master_boot, rf_seq, 0, 0);
            rf_expected = APP_MSG_TELEMETRY;
        }

        rf_attempts = 0;
        rf_state = RF_SEND;
        return;
    }

    np = &nodes[0];
    for (index = 0; index < NODE_COUNT; index++) {
        if (nodes[index].id == rf_node) {
            np = &nodes[index];
        }
    }

    if (rf_state == RF_SEND && (int32_t)(now - next_try) >= 0) {
        uint8_t a[5];
        node_addr(rf_node, a);
        rf_attempts++;

        if (!nrf24_send(a, frame, 32, 30)) {
            dbg("RF NO_HW_ACK node=");
            dbg_u32(rf_node);
            dbg(" try=");
            dbg_u32(rf_attempts);
            dbg("\r\n");

            if (rf_attempts >= 3) {
                node_result(np, 0);
                if (rf_command_active) np->next_poll = now + 1000u;
                rf_command_active = 0u;
                rf_state = RF_IDLE;
            } else {
                next_try = now + 20;
            }
            return;
        }

        nrf24_start_listening(MASTER_ADDR);
        rf_deadline = now + 150;
        rf_state = RF_WAIT;
    }

    if (rf_state == RF_WAIT) {
        if (nrf24_receive(rx, &len) &&
            app_validate_frame(rx, len, &h, &p) &&
            h->src_id == rf_node &&
            h->dst_id == 0 &&
            h->type == rf_expected) {

            if (h->type == APP_MSG_TELEMETRY && h->payload_len == sizeof(Telemetry)) {
                Telemetry t;
                memcpy(&t, p, sizeof(t));

                if (t.request_seq != rf_seq) {
                    dbg("RF DROP stale telemetry request_seq=");
                    dbg_u32(t.request_seq);
                    dbg(" expected=");
                    dbg_u32(rf_seq);
                    dbg("\r\n");
                    return;
                }

                if (np->has_seq && np->boot == h->boot_id && np->last_seq == h->seq) {
                    dbg("RF DUP telemetry boot=");
                    dbg_u32(h->boot_id);
                    dbg(" seq=");
                    dbg_u32(h->seq);
                    dbg("\r\n");
                } else {
                    memcpy(&np->data, &t, sizeof(t));
                    np->boot = h->boot_id;
                    np->last_seq = h->seq;
                    np->has_seq = 1u;
                    publish_telemetry(np, h, &np->data);
                }

                dbg("DATA n=");
                dbg_u32(rf_node);
                dbg(" T=");
                dbg_u32(t.temperature_centi_c);
                dbg(" H=");
                dbg_u32(t.humidity_centi_rh);
                dbg(" P=");
                dbg_u32(t.pressure_pa);
                dbg("\r\n");
            } else if (h->type == APP_MSG_COMMAND_ACK && h->payload_len == sizeof(CommandAck)) {
                CommandAck a;
                memcpy(&a, p, sizeof(a));

                if (rf_cmd_id && a.command_id != rf_cmd_id) {
                    dbg("RF DROP stale ACK id=");
                    dbg_u32(a.command_id);
                    dbg(" expected=");
                    dbg_u32(rf_cmd_id);
                    dbg("\r\n");
                    return;
                }

                if (rf_command_active && cc) {
                    cmd_remove_slot(rf_cmd_slot);
                    rf_command_active = 0u;
                }

                publish_command_result(rf_node, &a);
                dbg("RF CMD ACK node=");
                dbg_u32(rf_node);
                dbg(" id=");
                dbg_u32(a.command_id);
                dbg(" result=");
                dbg_u32(a.result);
                dbg(" value=");
                dbg_u32(a.value);
                dbg("\r\n");
            }

            node_result(np, 1);
            rf_state = RF_IDLE;
            return;
        }

        if ((int32_t)(now - rf_deadline) >= 0) {
            if (rf_attempts < 3) {
                rf_state = RF_SEND;
                next_try = now + 20;
            } else {
                dbg("RF NO_APP_RESPONSE command retained for retry\r\n");
                node_result(np, 0);
                if (rf_command_active) np->next_poll = now + 1000u;
                rf_command_active = 0u;
                rf_state = RF_IDLE;
            }
        }
    }
}

/* 读取 USB-TTL 输入并透传任意 AT 命令到 CLR970。 */
static void pc_task(void) {
    char c;

    while (readc(USART1, &c)) {
        if (c == '\r') continue;
        if (c == '\n') {
            if (pc_len) {
                pc_line[pc_len] = 0;
                clm_send(pc_line);
                pc_len = 0;
            }
        } else if (pc_len < sizeof(pc_line) - 1) {
            pc_line[pc_len++] = c;
        }
    }
}

/* 主函数：初始化 nRF24、串口并运行网关的双状态机。 */
int main(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000u);
    uart_init();

    dbg("\r\nSTM32 MASTER GATEWAY BOOT\r\n");
    dbg("CFG FW=");
    dbg(GATEWAY_FW_VERSION);
    dbg("\r\nCFG APN=");
    dbg(effective_apn());
    dbg("\r\nCFG MQTT=");
    dbg(MQTT_HOST);
    dbg(":");
    dbg_u32(MQTT_PORT);
    dbg(" CID=");
    dbg(effective_client_id());
    dbg(" ROOT=");
    dbg(MASTER_MQTT_ROOT);
    dbg("\r\nCFG CLM_BAUD=");
    dbg_u32(CLM_BAUD);
    dbg(" BOOT_DELAY_MS=");
    dbg_u32(CLM_BOOT_DELAY_MS);
    dbg("\r\n");

    if (!MQTT_USER[0]) {
        dbg("WARN MQTT_USER/PASSWORD EMPTY: public broker only; EMQX Cloud requires credentials\r\n");
    }

    master_boot = (uint16_t)(SysTick->VAL ^ 0xA55Au);

    {
        uint8_t ok = 0;
        uint8_t try;
        for (try = 1; try <= 3 && !ok; ++try) {
            dbg("NRF INIT try=");
            dbg_u32(try);
            dbg("\r\n");
            nrf24_init();
            nrf_dump();
            ok = nrf24_is_present();
            if (!ok) {
                dbg("NRF SELFTEST FAIL\r\n");
                for (volatile uint32_t d = 0; d < 800000u; ++d) __NOP();
            }
        }

        if (!ok) {
            dbg("FATAL NRF SPI - CHECK VCC/CSN/CE/SPI\r\n");
            for (;;) {
                nrf_dump();
                for (volatile uint32_t d = 0; d < 4000000u; ++d) __NOP();
            }
        }
    }

    dbg("NRF PASS RF_CH=");
    dbg_u32(nrf24_read_register(0x05));
    dbg(" RF_SETUP=0x");
    dbg_hex(nrf24_read_register(0x06));
    dbg(" RETR=0x");
    dbg_hex(nrf24_read_register(0x04));
    dbg("\r\n");

    for (uint8_t i = 0; i < NODE_COUNT; i++) {
        nodes[i].id = NODE_IDS[i];
        nodes[i].state = NODE_UNKNOWN;
        nodes[i].next_poll = 1000u + i * 200u;
    }

    dbg("CLM WAIT BOOT\r\n");
    clm_next = millis() + CLM_BOOT_DELAY_MS;

    for (;;) {
        clm_task();
        radio_task();
        pc_task();
    }
}

