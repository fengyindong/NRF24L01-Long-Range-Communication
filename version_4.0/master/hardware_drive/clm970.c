#include "clm970.h"

#include "board.h"
#include "master_config.h"

#include <stdio.h>
#include <string.h>

/* ============================================================
 * 配置常量
 * ============================================================ */

#define CLM_QUEUE_SIZE   8u      /* MQTT 发送队列长度 */
#define CLM_TOPIC_MAX   160u     /* 主题最大长度 */
#define CLM_PAYLOAD_MAX 512u     /* 载荷最大长度 */
#define CLM_LINE_MAX    768u     /* 单行 URC/响应最大长度 */
#define CLM_COMMAND_MAX 384u     /* AT 指令拼接缓冲区最大长度 */

/* ============================================================
 * 内部数据结构
 * ============================================================ */

/* MQTT 发送队列项 */
typedef struct {
    char     topic[CLM_TOPIC_MAX];       /* 主题 */
    char     payload[CLM_PAYLOAD_MAX];   /* 载荷内容 */
    uint16_t length;                     /* 载荷实际长度 */
    uint8_t  qos;                        /* QoS 等级 */
    uint8_t  retained;                   /* 是否保留消息 */
    uint8_t  used;                       /* 该槽位是否有效 */
} ClmPublishItem;

/* 模块初始化与 MQTT 连接状态机 */
typedef enum {
    CLM_STATE_BOOT_WAIT,      /* 等待模块启动完成 */
    CLM_STATE_AT,             /* 测试 AT 通信 */
    CLM_STATE_VERBOSE,        /* 开启详细错误 */
    CLM_STATE_SIM_QUERY,      /* 查询 SIM 卡 */
    CLM_STATE_SIGNAL_QUERY,   /* 查询信号质量 */
    CLM_STATE_REG_QUERY,      /* 查询网络注册状态 */
    CLM_STATE_ATTACH_QUERY,   /* 查询 PS 附着状态 */
    CLM_STATE_PDP_QUERY,      /* 查询 PDP 上下文 */
    CLM_STATE_PDP_CONFIG,     /* 配置 APN */
    CLM_STATE_ATTACH,         /* 请求 PS 附着 */
    CLM_STATE_ACTIVATE,       /* 激活 PDP */
    CLM_STATE_MQTT_CONFIG,    /* 配置 MQTT 参数（分 4 步） */
    CLM_STATE_CONNECT,        /* 连接 MQTT 服务器 */
    CLM_STATE_SUBSCRIBE,      /* 订阅命令主题 */
    CLM_STATE_READY,          /* 就绪，可收发数据 */
    CLM_STATE_BACKOFF         /* 退避重连 */
} ClmState;

/* 等待响应类型 */
typedef enum {
    CLM_WAIT_NONE,            /* 无等待 */
    CLM_WAIT_OK,              /* 等待 OK */
    CLM_WAIT_CONNECT_URC,     /* 等待连接 URC */
    CLM_WAIT_SUB_URC,         /* 等待订阅 URC */
    CLM_WAIT_PUB_PROMPT,      /* 等待 PUBEX 的 '>' 提示符 */
    CLM_WAIT_PUB_URC,         /* 等待发布结果 URC */
    CLM_WAIT_STATE_URC        /* 等待状态查询 URC */
} ClmWait;

/* ============================================================
 * 模块全局状态
 * ============================================================ */

static ClmPublishItem publish_queue[CLM_QUEUE_SIZE]; /* 发送队列 */
static uint8_t publish_head;                         /* 队头指针 */
static uint8_t publish_count;                        /* 队列当前长度 */

static ClmState state;                               /* 状态机当前状态 */
static ClmWait  wait_kind;                           /* 当前等待的响应类型 */
static uint8_t  waiting;                             /* 是否正在等待响应 */

static uint8_t mqtt_connected;                       /* MQTT 已连接 */
static uint8_t mqtt_subscribed;                      /* MQTT 已订阅 */
static uint8_t sim_ready;                            /* SIM 卡就绪 */
static uint8_t registered;                           /* 网络已注册 */
static uint8_t cereg_status;                         /* 最近一次 CEREG 状态值 */
static uint8_t attached;                             /* PS 已附着 */
static uint8_t pdp_active;                           /* PDP 已激活 */
static uint8_t mqtt_config_index;                    /* MQTT 配置步骤索引 */
static uint8_t backoff_count;                        /* 退避次数 */

static uint32_t next_action;                         /* 下一次操作时间 */
static uint32_t deadline;                            /* 等待响应超时时间 */
static uint32_t state_check_at;                      /* 下一次状态检查时间 */

static char     line_buffer[CLM_LINE_MAX];           /* 行缓冲 */
static char     incoming_payload[CLM_PAYLOAD_MAX];   /* 下行载荷缓冲（独立于队列） */
static uint16_t line_length;                         /* 行缓冲当前长度 */

static Clm970MessageHandler message_handler;         /* 下行消息回调 */

/* ============================================================
 * 工具函数
 * ============================================================ */

/**
 * @brief 判断当前时间是否已达到或超过指定时间戳。
 *        使用有符号差值，兼容 millis() 溢出回绕。
 */
static uint8_t time_due(uint32_t now, uint32_t at)
{
    return (uint8_t)((int32_t)(now - at) >= 0);
}

/**
 * @brief 解析 AT 响应中的非负十进制整数，并返回数字后的第一个字符。
 *
 * 不使用 atoi/strtoul，避免不同 Keil 运行库配置产生不一致行为。
 *
 * @param text   待解析的字符串
 * @param end    输出参数，返回数字之后的第一个字符
 * @param value  输出参数，返回解析结果
 * @return 1 成功；0 失败
 */
static uint8_t parse_decimal_u32(const char *text, const char **end,
                                 uint32_t *value)
{
    uint32_t parsed = 0u;
    uint8_t  digit;

    if (!text || !end || !value) return 0u;

    /* 跳过前导空白 */
    while (*text == ' ' || *text == '\t') ++text;
    if (*text < '0' || *text > '9') return 0u;

    /* 逐位解析，并检查是否溢出 */
    while (*text >= '0' && *text <= '9') {
        digit = (uint8_t)(*text - '0');
        if (parsed > (0xFFFFFFFFu - digit) / 10u) return 0u;
        parsed = parsed * 10u + digit;
        ++text;
    }

    *end   = text;
    *value = parsed;
    return 1u;
}

/**
 * @brief 解析一个 AT 整数字段，失败或超出 int 正数范围时返回 -1。
 */
static int parse_at_int(const char *text)
{
    const char *end;
    uint32_t value;

    if (!parse_decimal_u32(text, &end, &value) || value > 0x7FFFFFFFu) {
        return -1;
    }
    return (int)value;
}

/* 向调试串口输出文本 */
static void debug_text(const char *text)
{
    board_debug_write(text);
}

/* 输出带前缀的调试事件 */
static void debug_event(const char *prefix, const char *value)
{
    char text[192];
    (void)snprintf(text, sizeof(text), "%s%s\r\n", prefix, value ? value : "");
    debug_text(text);
}

/* ============================================================
 * CEREG 解析
 * ============================================================ */

/**
 * @brief 解析 +CEREG 响应中的 stat 字段。
 *
 * 兼容以下两种格式：
 *   简短格式：+CEREG: <n>,<stat>
 *   扩展格式：+CEREG: <n>,<stat>[,<lac>,<ci>,...]
 *
 * @param line   指向行字符串
 * @param status 输出参数，返回 stat 值
 * @return 1 成功；0 失败
 */
static uint8_t parse_cereg_status(char *line, int *status)
{
    char *cursor;
    char *comma;
    int   value;

    /* 定位冒号后第一个数字（跳过 <n>） */
    cursor = strchr(line, ':');
    if (!cursor) return 0u;
    ++cursor;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;

    if (*cursor < '0' || *cursor > '9') return 0u;
    value = 0;
    while (*cursor >= '0' && *cursor <= '9') {
        value = value * 10 + (*cursor - '0');
        ++cursor;
    }
    while (*cursor == ' ' || *cursor == '\t') ++cursor;

    /* 如果后面跟着逗号，则继续读取下一个数字作为 stat */
    comma = cursor;
    if (*comma == ',') {
        cursor = comma + 1;
        while (*cursor == ' ' || *cursor == '\t') ++cursor;
        if (*cursor < '0' || *cursor > '9') return 0u;
        value = 0;
        while (*cursor >= '0' && *cursor <= '9') {
            value = value * 10 + (*cursor - '0');
            ++cursor;
        }
    }

    *status = value;
    return 1u;
}

/**
 * @brief CEREG 解析失败时，打印原始字节（HEX）方便排查不可见字符。
 */
static void debug_cereg_parse_failed(const char *line)
{
    char   text[192];
    size_t index;
    size_t length = strlen(line);
    int    offset;

    offset = snprintf(text, sizeof(text), "CLM CEREG parse failed len=%u raw=",
                      (unsigned)length);

    for (index = 0u;
         index < length && index < 32u && offset > 0 &&
         offset < (int)sizeof(text) - 5;
         ++index) {
        offset += snprintf(text + offset, sizeof(text) - (size_t)offset,
                           "%02X ", (unsigned char)line[index]);
    }

    if (offset > 0 && offset < (int)sizeof(text) - 3) {
        (void)snprintf(text + offset, sizeof(text) - (size_t)offset, "\r\n");
    }
    debug_text(text);
}

/* ============================================================
 * AT 指令发送
 * ============================================================ */

/**
 * @brief 发送 AT 指令，自动追加 CRLF，并设置等待响应类型与超时。
 *
 * @param command    AT 指令（不含 CRLF）
 * @param expected   期望的响应类型
 * @param timeout_ms 超时时间（毫秒）
 */
static void send_command(const char *command, ClmWait expected,
                         uint32_t timeout_ms)
{
    char text[CLM_COMMAND_MAX];
    int  length;

    /* 拼接完整命令（含 CRLF） */
    length = snprintf(text, sizeof(text), "%s\r\n", command);
    if (length <= 0 || length >= (int)sizeof(text)) {
        return;
    }

    /* 仅对关键命令或 verbose 模式打印发送日志 */
    if (CLM_DEBUG_VERBOSE || strstr(command, "IMQTT") != NULL) {
        debug_event("CLM >> ", command);
    }

    board_clm_write((const uint8_t *)text, (uint16_t)length);

    wait_kind = expected;
    waiting   = 1u;
    deadline  = board_millis() + timeout_ms;
}

/**
 * @brief 进入指数退避状态，等待一段时间后重新初始化。
 *
 * 退避时间随失败次数翻倍，但不超过 CLM_BACKOFF_MAX_MS。
 * 发送队列不清空，网络恢复后可继续发送。
 */
static void enter_backoff(const char *reason)
{
    uint32_t delay_ms = CLM_RECONNECT_MS;
    uint8_t  index;

    /* 指数退避计算 */
    for (index = 0u;
         index < backoff_count && delay_ms < CLM_BACKOFF_MAX_MS / 2u;
         ++index) {
        delay_ms *= 2u;
    }
    if (delay_ms > CLM_BACKOFF_MAX_MS) delay_ms = CLM_BACKOFF_MAX_MS;
    if (backoff_count < 6u) ++backoff_count;

    /* 重置连接状态 */
    mqtt_connected  = 0u;
    mqtt_subscribed = 0u;
    waiting         = 0u;
    wait_kind       = CLM_WAIT_NONE;
    state           = CLM_STATE_BACKOFF;
    next_action     = board_millis() + delay_ms;

    /* 打印退避原因与诊断信息 */
    {
        char text[160];
        (void)snprintf(text, sizeof(text),
                       "CLM BACKOFF reason=%s delay_ms=%lu rx=%lu err=%lu\r\n",
                       reason, (unsigned long)delay_ms,
                       (unsigned long)board_clm_rx_bytes(),
                       (unsigned long)board_clm_uart_errors());
        debug_text(text);
    }
}

/**
 * @brief 判断该主题是否属于“只保留最新值”的类型（遥测、状态）。
 */
static uint8_t topic_is_latest_value(const char *topic)
{
    return (uint8_t)(strstr(topic, "/telemetry") != NULL ||
                     strstr(topic, "/state") != NULL);
}

/* ============================================================
 * 公共接口：MQTT 发布入队
 * ============================================================ */

/**
 * @brief 将一条 MQTT 消息加入发送队列。
 *
 * 对遥测/状态类主题做去重：若队列中已有同主题消息，直接覆盖其内容，
 * 但绝不覆盖正在发送的队首（避免丢包）。
 *
 * @param topic    主题
 * @param payload  载荷
 * @param qos      QoS
 * @param retained 是否保留
 * @return 1 成功；0 失败（参数错误或队列满）
 */
uint8_t clm970_publish(const char *topic, const char *payload,
                       uint8_t qos, uint8_t retained)
{
    uint8_t index;
    uint8_t queue_index;
    size_t  topic_length;
    size_t  payload_length;

    if (!topic || !payload) return 0u;

    topic_length   = strlen(topic);
    payload_length = strlen(payload);

    if (topic_length == 0u || topic_length >= CLM_TOPIC_MAX ||
        payload_length >= CLM_PAYLOAD_MAX) {
        return 0u;
    }

    /* 同主题遥测/状态仅保留最新值，但绝不覆盖正在发送的队首 */
    if (topic_is_latest_value(topic)) {
        for (index = 0u; index < publish_count; ++index) {
            queue_index = (uint8_t)((publish_head + index) % CLM_QUEUE_SIZE);

            /* 跳过正在等待发布结果的队首 */
            if (index == 0u && wait_kind == CLM_WAIT_PUB_URC) continue;

            if (publish_queue[queue_index].used &&
                strcmp(publish_queue[queue_index].topic, topic) == 0) {
                memcpy(publish_queue[queue_index].payload, payload,
                       payload_length + 1u);
                publish_queue[queue_index].length   = (uint16_t)payload_length;
                publish_queue[queue_index].qos      = qos;
                publish_queue[queue_index].retained = retained;
                return 1u;
            }
        }
    }

    if (publish_count >= CLM_QUEUE_SIZE) return 0u;

    /* 入队到队尾 */
    queue_index = (uint8_t)((publish_head + publish_count) % CLM_QUEUE_SIZE);
    memcpy(publish_queue[queue_index].topic, topic, topic_length + 1u);
    memcpy(publish_queue[queue_index].payload, payload, payload_length + 1u);
    publish_queue[queue_index].length   = (uint16_t)payload_length;
    publish_queue[queue_index].qos      = qos;
    publish_queue[queue_index].retained = retained;
    publish_queue[queue_index].used     = 1u;
    ++publish_count;
    return 1u;
}

/* ============================================================
 * 下行消息解析
 * ============================================================ */

/**
 * @brief 解析 +IMQTTRCVPUB 上报内容，并回调给上层。
 *
 * 格式：+IMQTTRCVPUB:<id>,"<topic>",<len>,"<payload>"
 *
 * 注意：使用独立的 incoming_payload 缓冲，避免污染发布队列。
 * CLR970 某些固件会把 JSON 双引号转义为 \"，这里做反转义处理。
 */
static void parse_incoming_publish(char *line)
{
    char       *cursor = line + 13;
    char       *separator;
    char       *topic;
    char       *topic_end;
    const char *length_end;
    char       *read_ptr;
    char       *write_ptr;
    uint32_t    payload_length;
    uint32_t    decoded_length;

    /* 跳过 <id>，定位到主题 */
    separator = strchr(cursor, ',');
    if (!separator) return;
    topic = separator + 1;

    /* 去掉主题两端的双引号 */
    if (*topic == '"') ++topic;
    topic_end = strchr(topic, '"');
    if (!topic_end) return;
    *topic_end = '\0';

    /* 解析载荷长度 */
    separator = strchr(topic_end + 1, ',');
    if (!separator) return;
    if (!parse_decimal_u32(separator + 1, &length_end, &payload_length) ||
        *length_end != ',') {
        return;
    }
    if (payload_length >= CLM_PAYLOAD_MAX) return;

    /* 定位载荷并反转义 \" */
    read_ptr = (char *)(length_end + 1);
    if (*read_ptr == '"') ++read_ptr;

    write_ptr      = incoming_payload;
    decoded_length = 0u;
    while (*read_ptr && decoded_length < payload_length) {
        if (read_ptr[0] == '\\' && read_ptr[1] == '"') {
            ++read_ptr;   /* 跳过反斜杠，只保留双引号 */
        }
        *write_ptr++ = *read_ptr++;
        ++decoded_length;
    }

    /* 长度不符说明数据不完整，直接丢弃 */
    if (decoded_length != payload_length) return;
    *write_ptr = '\0';

    if (message_handler) message_handler(topic, incoming_payload);
}

/* ============================================================
 * OK 响应处理：推进状态机
 * ============================================================ */

static void handle_ok(void)
{
    if (!waiting || wait_kind != CLM_WAIT_OK) return;

    waiting     = 0u;
    wait_kind   = CLM_WAIT_NONE;
    next_action = board_millis() + 200u;

    switch (state) {
        case CLM_STATE_AT:
            state = CLM_STATE_VERBOSE;
            break;

        case CLM_STATE_VERBOSE:
            state = CLM_STATE_SIM_QUERY;
            break;

        case CLM_STATE_SIM_QUERY:
            if (sim_ready) state = CLM_STATE_SIGNAL_QUERY;
            else           enter_backoff("SIM_NOT_READY");
            break;

        case CLM_STATE_SIGNAL_QUERY:
            state = CLM_STATE_REG_QUERY;
            break;

        case CLM_STATE_REG_QUERY:
            if (registered) {
                state = CLM_STATE_ATTACH_QUERY;
            } else if (cereg_status == 0u || cereg_status == 2u ||
                       cereg_status == 4u || cereg_status == 99u ||
                       cereg_status == 0xFFu) {
                /* 尚未注册完成，稍后继续查询 */
                state       = CLM_STATE_REG_QUERY;
                next_action = board_millis() + CLM_REG_POLL_MS;
                debug_text("CLM NET WAIT REGISTRATION\r\n");
            } else {
                enter_backoff("NOT_REGISTERED");
            }
            break;

        case CLM_STATE_ATTACH_QUERY:
            state = attached ? CLM_STATE_PDP_QUERY : CLM_STATE_PDP_CONFIG;
            break;

        case CLM_STATE_PDP_QUERY:
            state = pdp_active ? CLM_STATE_MQTT_CONFIG : CLM_STATE_PDP_CONFIG;
            mqtt_config_index = 0u;
            break;

        case CLM_STATE_PDP_CONFIG:
            state = CLM_STATE_ATTACH;
            break;

        case CLM_STATE_ATTACH:
            state = CLM_STATE_ACTIVATE;
            break;

        case CLM_STATE_ACTIVATE:
            state = CLM_STATE_MQTT_CONFIG;
            mqtt_config_index = 0u;
            break;

        case CLM_STATE_MQTT_CONFIG:
            ++mqtt_config_index;
            if (mqtt_config_index >= 4u) state = CLM_STATE_CONNECT;
            break;

        default:
            break;
    }
}

/* ============================================================
 * 行解析：处理模块返回的每一行
 * ============================================================ */

static void handle_line(char *line)
{
    char *comma;
    int   value;
    char  diagnostic[192];

    if (line[0] == '\0') return;

    /* 去掉行首空白 */
    while (*line == ' ' || *line == '\t') ++line;
    if (line[0] == '\0') return;

    /* 关键行日志输出 */
    if (CLM_DEBUG_VERBOSE || strstr(line, "+IMQTT") != NULL ||
        strstr(line, "ERROR") != NULL) {
        debug_event("CLM << ", line);
    }

    /* 模块重启检测 */
    if (strcmp(line, "RDY") == 0 || strstr(line, "ATREADY") != NULL) {
        mqtt_connected  = 0u;
        mqtt_subscribed = 0u;
        waiting         = 0u;
        wait_kind       = CLM_WAIT_NONE;
        state           = CLM_STATE_BOOT_WAIT;
        next_action     = board_millis() + CLM_BOOT_DELAY_MS;
        debug_text("CLM REBOOT DETECTED\r\n");
        return;
    }

    if (strcmp(line, "OK") == 0) {
        handle_ok();
        return;
    }

    if (strstr(line, "ERROR") != NULL) {
        enter_backoff("AT_ERROR");
        return;
    }

    /* SIM 卡状态 */
    if (strncmp(line, "+CPIN:", 6u) == 0) {
        sim_ready = (uint8_t)(strstr(line, "READY") != NULL);
        if (sim_ready) debug_text("CLM SIM READY\r\n");
        return;
    }

    /* 网络注册状态 */
    if (strncmp(line, "+CEREG:", 7u) == 0) {
        if (parse_cereg_status(line, &value)) {
            cereg_status = (uint8_t)value;
            registered   = (uint8_t)(value == 1 || value == 5);
            (void)snprintf(diagnostic, sizeof(diagnostic),
                           "CLM CEREG status=%d registered=%u raw=[%s]\r\n",
                           value, (unsigned)registered, line);
            debug_text(diagnostic);
        } else {
            cereg_status = 0xFFu;
            registered   = 0u;
            debug_cereg_parse_failed(line);
        }
        return;
    }

    /* PS 附着状态 */
    if (strncmp(line, "+CGATT:", 7u) == 0) {
        attached = (uint8_t)(parse_at_int(line + 7) == 1);
        if (attached) debug_text("CLM NET ATTACHED\r\n");
        return;
    }

    /* PDP 上下文状态 */
    if (strncmp(line, "+CGACT:", 7u) == 0) {
        char *cursor = line + 7;
        while (*cursor == ' ') ++cursor;
        comma = strchr(cursor, ',');
        pdp_active = (uint8_t)(parse_at_int(cursor) == 1 && comma &&
                               parse_at_int(comma + 1) == 1);
        if (pdp_active) debug_text("CLM PDP ACTIVE\r\n");
        return;
    }

    /* MQTT 连接结果 */
    if (strncmp(line, "+IMQTTCONN:", 11u) == 0) {
        value = parse_at_int(line + 11);
        waiting   = 0u;
        wait_kind = CLM_WAIT_NONE;

        if (value == 0 || value == 1) {
            mqtt_connected = 1u;
            backoff_count  = 0u;
            state          = CLM_STATE_SUBSCRIBE;
            next_action    = board_millis() + 200u;
            debug_text("MQTT CONNECTED\r\n");
        } else {
            enter_backoff("MQTT_CONNECT_FAIL");
        }
        return;
    }

    /* MQTT 订阅结果 */
    if (strncmp(line, "+IMQTTSUB:", 10u) == 0) {
        comma = strrchr(line, ',');
        value = comma ? parse_at_int(comma + 1) : -1;
        waiting   = 0u;
        wait_kind = CLM_WAIT_NONE;

        if (value == 0) {
            mqtt_subscribed = 1u;
            backoff_count   = 0u;
            state           = CLM_STATE_READY;
            state_check_at  = board_millis() + CLM_STATE_CHECK_MS;
            debug_text("MQTT SUBSCRIBED\r\n");
        } else {
            enter_backoff("MQTT_SUB_FAIL");
        }
        return;
    }

    /* MQTT 状态查询结果 */
    if (strncmp(line, "+IMQTTSTATE:", 12u) == 0) {
        value = parse_at_int(line + 12);
        waiting   = 0u;
        wait_kind = CLM_WAIT_NONE;

        if (value == 1 || value == 2) {
            mqtt_connected = 1u;
            state = mqtt_subscribed ? CLM_STATE_READY : CLM_STATE_SUBSCRIBE;
            next_action    = board_millis() + 200u;
            state_check_at = board_millis() + CLM_STATE_CHECK_MS;
        } else {
            enter_backoff("MQTT_STATE_OFFLINE");
        }
        return;
    }

    /* MQTT 发布结果 */
    if (strncmp(line, "+IMQTTPUBEX:", 12u) == 0 &&
        wait_kind == CLM_WAIT_PUB_URC) {
        comma = strrchr(line, ',');
        value = comma ? parse_at_int(comma + 1) : -1;
        waiting   = 0u;
        wait_kind = CLM_WAIT_NONE;

        if (value == 0 && publish_count) {
            publish_queue[publish_head].used = 0u;
            publish_head = (uint8_t)((publish_head + 1u) % CLM_QUEUE_SIZE);
            --publish_count;
            state       = CLM_STATE_READY;
            next_action = board_millis() + 50u;
            debug_text("MQTT PUB OK\r\n");
        } else {
            enter_backoff("MQTT_PUB_FAIL");
        }
        return;
    }

    /* MQTT 收到下行消息 */
    if (strncmp(line, "+IMQTTRCVPUB:", 13u) == 0) {
        parse_incoming_publish(line);
    }
}

/* ============================================================
 * 串口接收任务：逐字节读取并按行解析
 * ============================================================ */

static void receive_task(void)
{
    uint8_t byte;

    while (board_clm_read(&byte)) {
        /* PUBEX 的 '>' 提示符：立即发送已入队的数据 */
        if (byte == '>' && waiting && wait_kind == CLM_WAIT_PUB_PROMPT &&
            publish_count) {
            board_clm_write((const uint8_t *)publish_queue[publish_head].payload,
                            publish_queue[publish_head].length);
            wait_kind = CLM_WAIT_PUB_URC;
            deadline  = board_millis() + CLM_PUBLISH_TIMEOUT_MS;
            continue;
        }

        if (byte == '\r') continue;

        if (byte == '\n') {
            if (line_length) {
                line_buffer[line_length] = '\0';
                handle_line(line_buffer);
                line_length = 0u;
            }
        } else if (line_length + 1u < sizeof(line_buffer)) {
            line_buffer[line_length++] = (char)byte;
        } else {
            /* 行过长，丢弃整行，防止溢出 */
            line_length = 0u;
        }
    }
}

/* ============================================================
 * 根据当前状态发送对应的 AT 指令
 * ============================================================ */

static void issue_state_command(void)
{
    char command[CLM_COMMAND_MAX];

    switch (state) {
        case CLM_STATE_AT:
            send_command("AT", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_STATE_VERBOSE:
            send_command("AT+CMEE=2", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_STATE_SIM_QUERY:
            sim_ready = 0u;
            send_command("AT+CPIN?", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_STATE_SIGNAL_QUERY:
            send_command("AT+CSQ", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_STATE_REG_QUERY:
            registered   = 0u;
            cereg_status = 0xFFu;
            send_command("AT+CEREG?", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_STATE_ATTACH_QUERY:
            attached = 0u;
            send_command("AT+CGATT?", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_STATE_PDP_QUERY:
            pdp_active = 0u;
            send_command("AT+CGACT?", CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_STATE_PDP_CONFIG:
            (void)snprintf(command, sizeof(command), "AT+CGDCONT=1,\"IP\",\"%s\"",
                           CLM_APN);
            send_command(command, CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_STATE_ATTACH:
            send_command("AT+CGATT=1", CLM_WAIT_OK, 15000u);
            break;

        case CLM_STATE_ACTIVATE:
            send_command("AT+CGACT=1,1", CLM_WAIT_OK, 15000u);
            break;

        case CLM_STATE_MQTT_CONFIG:
            /* MQTT 配置分 4 步，按 mqtt_config_index 依次发送 */
            if (mqtt_config_index == 0u) {
                (void)snprintf(command, sizeof(command), "AT+IMQTTMODE=2,%u,0",
                               (unsigned)CLM_MQTT_TLS);
            } else if (mqtt_config_index == 1u) {
                (void)snprintf(command, sizeof(command),
                               "AT+IMQTTPARA=TIMEOUT,2,CLEAN,1,KEEPALIVE,60,VERSION,3.1.1");
            } else if (mqtt_config_index == 2u) {
                (void)snprintf(command, sizeof(command),
                               "AT+IMQTTADDR=\"%s\",%u,\"%s\"", CLM_MQTT_HOST,
                               (unsigned)CLM_MQTT_PORT, CLM_MQTT_CLIENT_ID);
            } else {
                (void)snprintf(command, sizeof(command),
                               "AT+IMQTTUSER=\"%s\",\"%s\"", CLM_MQTT_USER,
                               CLM_MQTT_PASSWORD);
            }
            send_command(command, CLM_WAIT_OK, CLM_COMMAND_TIMEOUT_MS);
            break;

        case CLM_STATE_CONNECT:
            send_command("AT+IMQTTCONN", CLM_WAIT_CONNECT_URC,
                         CLM_CONNECT_TIMEOUT_MS);
            break;

        case CLM_STATE_SUBSCRIBE:
            (void)snprintf(command, sizeof(command),
                           "AT+IMQTTSUB=%s/node/+/command/set,%u",
                           GATEWAY_MQTT_ROOT, (unsigned)CLM_MQTT_SUB_QOS);
            send_command(command, CLM_WAIT_SUB_URC, CLM_COMMAND_TIMEOUT_MS);
            break;

        default:
            break;
    }
}

/* ============================================================
 * 公共接口实现
 * ============================================================ */

/**
 * @brief 初始化 CLM970 驱动。
 *
 * @param handler 下行消息回调，可为 NULL
 */
void clm970_init(Clm970MessageHandler handler)
{
    memset(publish_queue, 0, sizeof(publish_queue));
    publish_head  = 0u;
    publish_count = 0u;

    mqtt_connected  = 0u;
    mqtt_subscribed = 0u;
    sim_ready = registered = attached = pdp_active = 0u;
    cereg_status = 0xFFu;
    mqtt_config_index = 0u;
    backoff_count = 0u;

    waiting     = 0u;
    wait_kind   = CLM_WAIT_NONE;
    line_length = 0u;

    message_handler = handler;
    state = CLM_STATE_BOOT_WAIT;
    next_action = board_millis() + CLM_BOOT_DELAY_MS;

    debug_text("CLM WAIT BOOT\r\n");
}

/* 动态切换下行消息回调 */
void clm970_set_message_handler(Clm970MessageHandler handler)
{
    message_handler = handler;
}

/* 查询 MQTT 是否已连接并订阅就绪 */
uint8_t clm970_is_connected(void)
{
    return (uint8_t)(mqtt_connected && mqtt_subscribed);
}

/**
 * @brief CLM970 主任务函数，需在主循环中反复调用。
 *
 * 职责：
 *   1) 处理串口接收；
 *   2) 判断是否等待超时；
 *   3) 按状态机发送对应 AT 指令；
 *   4) 在 READY 状态下发送队列中的 MQTT 消息或定期查询状态。
 */
void clm970_task(void)
{
    uint32_t        now;
    char            command[CLM_COMMAND_MAX];
    ClmPublishItem *item;

    receive_task();
    now = board_millis();

    /* 等待响应超时则进入退避 */
    if (waiting) {
        if (!time_due(now, deadline)) return;
        enter_backoff("RESPONSE_TIMEOUT");
        return;
    }

    /* 未到下一次操作时间，直接返回 */
    if (!time_due(now, next_action)) return;

    /* 启动等待阶段：进入 AT */
    if (state == CLM_STATE_BOOT_WAIT) {
        state = CLM_STATE_AT;
        issue_state_command();
        return;
    }

    /* 退避结束：从 AT 重新开始 */
    if (state == CLM_STATE_BACKOFF) {
        state = CLM_STATE_AT;
        issue_state_command();
        return;
    }

    /* 未就绪状态：继续推进状态机 */
    if (state != CLM_STATE_READY) {
        issue_state_command();
        return;
    }

    /* 就绪状态下，检查连接是否仍然有效 */
    if (!mqtt_connected || !mqtt_subscribed) {
        enter_backoff("LOCAL_DISCONNECTED");
        return;
    }

    /* 有待发送消息：先发一条 */
    if (publish_count) {
        item = &publish_queue[publish_head];
        (void)snprintf(command, sizeof(command),
                       "AT+IMQTTPUBEX=%s,%u,%u,%u", item->topic,
                       (unsigned)item->qos, (unsigned)item->retained,
                       (unsigned)item->length);
        send_command(command, CLM_WAIT_PUB_PROMPT, CLM_PUBLISH_TIMEOUT_MS);
        return;
    }

    /* 定期查询 MQTT 状态 */
    if (time_due(now, state_check_at)) {
        send_command("AT+IMQTTSTATE?", CLM_WAIT_STATE_URC,
                     CLM_COMMAND_TIMEOUT_MS);
        return;
    }

    next_action = now + 200u;
}