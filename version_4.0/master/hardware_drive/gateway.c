#include "gateway.h"

#include <stdio.h>
#include <string.h>

#include "app_protocol.h"
#include "board.h"
#include "clm970.h"
#include "master_config.h"
#include "mqtt_command.h"
#include "nrf24.h"

/* ============================================================
 * 配置常量
 * ============================================================ */

#define GATEWAY_TOPIC_MAX    128u   /* MQTT 主题最大长度 */
#define GATEWAY_PAYLOAD_MAX  256u   /* MQTT 载荷最大长度 */

/* 主机接收地址；从机响应帧必须发往该地址。 */
static const uint8_t MASTER_ADDRESS[5] = {
    0xD2u, 0xD2u, 0xD2u, 0xD2u, 0xA0u
};

/* ============================================================
 * 内部数据结构
 * ============================================================ */

/* 命令队列槽位 */
typedef struct {
    MqttCommand command;   /* 解析后的命令 */
    uint8_t     used;      /* 该槽位是否有效 */
} GatewayCommandSlot;

/* 从机节点信息 */
typedef struct {
    uint8_t  id;           /* 节点 ID */
    uint32_t next_poll;    /* 下一次轮询时间 */
} GatewayNode;

/* 待发布 MQTT 消息 */
typedef struct {
    char    topic[GATEWAY_TOPIC_MAX];       /* 主题 */
    char    payload[GATEWAY_PAYLOAD_MAX];   /* 载荷 */
    uint8_t retained;                       /* 是否保留消息 */
    uint8_t used;                           /* 该槽位是否有效 */
} GatewayPublishItem;

/* 无线收发状态机 */
typedef enum {
    GATEWAY_RADIO_IDLE,    /* 空闲，可发起新请求 */
    GATEWAY_RADIO_WAIT     /* 已发出请求，等待从机响应 */
} GatewayRadioState;

/* ============================================================
 * 模块全局状态
 * ============================================================ */

static GatewayCommandSlot command_queue[GATEWAY_COMMAND_QUEUE_SIZE]; /* 命令队列 */
static GatewayNode        nodes[GATEWAY_NODE_COUNT];                 /* 节点列表 */
static GatewayPublishItem publish_queue[GATEWAY_PUBLISH_QUEUE_SIZE]; /* 发布队列 */

static uint8_t command_count;             /* 命令队列当前长度 */
static uint8_t publish_head;              /* 发布队列队头 */
static uint8_t publish_count;             /* 发布队列当前长度 */

static uint8_t  active_node;              /* 当前正在通信的节点 */
static uint8_t  active_expected_type;     /* 期望的响应类型 */
static uint16_t next_sequence;            /* 下一帧序列号 */

static uint16_t master_boot = 0xF407u;    /* 主机启动 ID（固定值） */
static uint32_t active_command_id;        /* 当前命令 ID */
static uint32_t application_deadline;     /* 应用层响应超时时间 */
static uint32_t next_radio_action;        /* 下一次无线操作时间 */
static GatewayRadioState radio_state;     /* 无线状态机 */

static uint8_t tx_frame[APP_RADIO_MAX_FRAME]; /* 发送帧缓冲区 */

/* ============================================================
 * 基础工具函数
 * ============================================================ */

/* 向调试串口输出网关日志 */
static void gateway_debug(const char *text)
{
    board_debug_write(text);
}

/**
 * @brief 判断当前时间是否已达到或超过指定时间戳。
 *        使用有符号差值，兼容 millis() 溢出回绕。
 */
static uint8_t due(uint32_t now, uint32_t then)
{
    return (uint8_t)((int32_t)(now - then) >= 0);
}

/**
 * @brief 根据节点 ID 生成 5 字节无线地址。
 *
 * 前 4 字节固定为 0xE7，最后 1 字节为节点 ID。
 */
static void node_address(uint8_t node, uint8_t address[5])
{
    address[0] = 0xE7u;
    address[1] = 0xE7u;
    address[2] = 0xE7u;
    address[3] = 0xE7u;
    address[4] = node;
}

/* ============================================================
 * 发布队列管理
 * ============================================================ */

/**
 * @brief 将一条 MQTT 消息加入发布队列。
 *
 * 队列满或参数超限时静默丢弃。
 *
 * @param topic    主题
 * @param payload  载荷
 * @param retained 是否保留消息
 */
static void queue_publish(const char *topic, const char *payload, uint8_t retained)
{
    GatewayPublishItem *item;
    uint8_t index;
    size_t  topic_length;
    size_t  payload_length;

    if (!topic || !payload || publish_count >= GATEWAY_PUBLISH_QUEUE_SIZE) {
        return;
    }

    topic_length   = strlen(topic);
    payload_length = strlen(payload);

    if (!topic_length || topic_length >= GATEWAY_TOPIC_MAX ||
        payload_length >= GATEWAY_PAYLOAD_MAX) {
        return;
    }

    index = (uint8_t)((publish_head + publish_count) % GATEWAY_PUBLISH_QUEUE_SIZE);
    item  = &publish_queue[index];

    memcpy(item->topic, topic, topic_length + 1u);
    memcpy(item->payload, payload, payload_length + 1u);
    item->retained = retained;
    item->used     = 1u;

    ++publish_count;
}

/* ============================================================
 * 发布具体内容
 * ============================================================ */

/**
 * @brief 发布命令执行结果。
 */
static void publish_command_result(uint8_t node, const AppCommandAck *ack)
{
    char topic[GATEWAY_TOPIC_MAX];
    char payload[GATEWAY_PAYLOAD_MAX];

    (void)snprintf(topic, sizeof(topic), "%s/node/%u/command/result",
                   GATEWAY_MQTT_ROOT, node);

    (void)snprintf(payload, sizeof(payload),
                   "{\"node_id\":%u,\"command_id\":%lu,\"code\":%u,\"result\":%u,\"value\":%lu}",
                   node, (unsigned long)ack->command_id, ack->command,
                   ack->result, (unsigned long)ack->value);

    queue_publish(topic, payload, 0u);
}

/**
 * @brief 发布继电器实际状态（retained，便于客户端重连后立即获取）。
 */
static void publish_relay_state(uint8_t node, const AppCommandAck *ack)
{
    char topic[GATEWAY_TOPIC_MAX];
    char payload[GATEWAY_PAYLOAD_MAX];

    (void)snprintf(topic, sizeof(topic), "%s/node/%u/relay/state",
                   GATEWAY_MQTT_ROOT, node);

    if (mqtt_relay_state_json(payload, sizeof(payload), node, ack->command_id,
                              (uint8_t)ack->value) > 0) {
        queue_publish(topic, payload, 1u);
    }
}

/**
 * @brief 发布遥测数据。
 *
 * 传感器通过 nRF24 传输整数定点值（温度单位 0.01℃），
 * 这里只在 MQTT 出口格式化为带单位字符串。
 *
 * 注意：不使用 %f，避免 Keil 精简 C 库未启用浮点 printf 时输出异常。
 */
static void publish_telemetry(uint8_t node, const AppTelemetry *telemetry)
{
    char topic[GATEWAY_TOPIC_MAX];
    char payload[GATEWAY_PAYLOAD_MAX];
    int32_t temperature = telemetry->temperature_centi_c;
    uint32_t temperature_abs;
    const char *temperature_sign;

    /* 温度正负号处理：负数取绝对值，符号单独保存 */
    if (temperature < 0) {
        temperature_sign = "-";
        temperature_abs  = (uint32_t)(-temperature);
    } else {
        temperature_sign = "";
        temperature_abs  = (uint32_t)temperature;
    }

    (void)snprintf(topic, sizeof(topic), "%s/node/%u/telemetry",
                   GATEWAY_MQTT_ROOT, node);

    /*
     * 同时输出“数值型”和“带单位的字符串型”字段，方便不同客户端解析：
     *   - temperature_c : 数值（单位 ℃）
     *   - temperature   : 带单位字符串（如 "23.45 °C"）
     *   - humidity_rh   : 数值（单位 %RH）
     *   - humidity      : 带单位字符串
     *   - pressure_hpa  : 数值（单位 hPa）
     *   - pressure      : 带单位字符串
     */
    (void)snprintf(payload, sizeof(payload),
                   "{\"node_id\":%u,"
                   "\"temperature_c\":%s%lu.%02lu,"
                   "\"temperature\":\"%s%lu.%02lu \\u00B0C\","
                   "\"humidity_rh\":%lu.%02lu,"
                   "\"humidity\":\"%lu.%02lu %%RH\","
                   "\"pressure_hpa\":%lu.%02lu,"
                   "\"pressure\":\"%lu.%02lu hPa\","
                   "\"sensor_status\":%u}",
                   node,
                   temperature_sign,
                   (unsigned long)(temperature_abs / 100u),
                   (unsigned long)(temperature_abs % 100u),
                   temperature_sign,
                   (unsigned long)(temperature_abs / 100u),
                   (unsigned long)(temperature_abs % 100u),
                   (unsigned long)(telemetry->humidity_centi_rh / 100u),
                   (unsigned long)(telemetry->humidity_centi_rh % 100u),
                   (unsigned long)(telemetry->humidity_centi_rh / 100u),
                   (unsigned long)(telemetry->humidity_centi_rh % 100u),
                   (unsigned long)(telemetry->pressure_pa / 100u),
                   (unsigned long)(telemetry->pressure_pa % 100u),
                   (unsigned long)(telemetry->pressure_pa / 100u),
                   (unsigned long)(telemetry->pressure_pa % 100u),
                   telemetry->sensor_status);

    queue_publish(topic, payload, 0u);
}

/* ============================================================
 * 命令队列管理
 * ============================================================ */

/**
 * @brief 移除命令队列的第一个命令（命令执行成功或放弃时调用）。
 */
static void remove_first_command(void)
{
    uint8_t index;

    if (!command_count) {
        return;
    }

    /* 将后续命令整体前移，填补空位 */
    for (index = 1u; index < command_count; ++index) {
        command_queue[index - 1u] = command_queue[index];
    }

    command_queue[command_count - 1u].used = 0u;
    --command_count;
}

/* ============================================================
 * 节点调度
 * ============================================================ */

/**
 * @brief 选择一个轮询时间已到的节点。
 *
 * @param now  当前时间
 * @param node 输出参数，返回选中的节点 ID
 * @return 1 表示找到节点；0 表示没有到期节点
 */
static uint8_t select_due_node(uint32_t now, uint8_t *node)
{
    uint8_t index;

    for (index = 0u; index < GATEWAY_NODE_COUNT; ++index) {
        if (due(now, nodes[index].next_poll)) {
            *node = nodes[index].id;
            return 1u;
        }
    }
    return 0u;
}

/**
 * @brief 重新安排指定节点的下一次轮询时间。
 */
static void schedule_next_poll(uint8_t node, uint32_t now)
{
    uint8_t index;

    for (index = 0u; index < GATEWAY_NODE_COUNT; ++index) {
        if (nodes[index].id == node) {
            nodes[index].next_poll = now + GATEWAY_NORMAL_POLL_MS;
            return;
        }
    }
}

/* ============================================================
 * 无线帧构建
 * ============================================================ */

/**
 * @brief 根据 MQTT 命令构造无线命令帧。
 */
static void start_command(const MqttCommand *command)
{
    AppCommand wire_command;

    wire_command.command_id = command->id;
    wire_command.command    = command->code;
    wire_command.reserved   = 0u;
    wire_command.value      = command->value;

    (void)app_build_frame(tx_frame, APP_MSG_COMMAND, APP_MASTER_ID,
                          command->node, master_boot, ++next_sequence,
                          &wire_command, sizeof(wire_command));

    active_node          = command->node;
    active_expected_type = APP_MSG_COMMAND_ACK;
    active_command_id    = command->id;
    next_radio_action    = board_millis();

    {
        char text[128];
        (void)snprintf(text, sizeof(text),
                       "RF CMD SEND node=%u id=%lu code=%u value=%lu\r\n",
                       command->node, (unsigned long)command->id,
                       command->code, (unsigned long)command->value);
        gateway_debug(text);
    }
}

/**
 * @brief 构造轮询请求帧。
 */
static void start_poll(uint8_t node)
{
    (void)app_build_frame(tx_frame, APP_MSG_POLL, APP_MASTER_ID, node,
                          master_boot, ++next_sequence, NULL, 0u);

    active_node          = node;
    active_expected_type = APP_MSG_TELEMETRY;
    active_command_id    = 0u;

    gateway_debug("RF POLL SEND\r\n");
}

/* ============================================================
 * 响应处理
 * ============================================================ */

/**
 * @brief 处理从机返回的响应（命令 ACK 或遥测数据）。
 */
static void finish_response(const AppHeader *header, const uint8_t *payload,
                            uint32_t now)
{
    /* ---------- 命令 ACK ---------- */
    if (active_expected_type == APP_MSG_COMMAND_ACK &&
        header->payload_len == sizeof(AppCommandAck)) {

        AppCommandAck ack;
        memcpy(&ack, payload, sizeof(ack));

        /* 命令 ID 不匹配则丢弃 */
        if (ack.command_id != active_command_id) {
            gateway_debug("RF CMD ACK DROP: command_id mismatch\r\n");
            return;
        }

        {
            char text[128];
            (void)snprintf(text, sizeof(text),
                           "RF CMD ACK node=%u id=%lu result=%u value=%lu\r\n",
                           active_node, (unsigned long)ack.command_id,
                           ack.result, (unsigned long)ack.value);
            gateway_debug(text);
        }

        publish_command_result(active_node, &ack);

        /* 收到合法应用 ACK 后，无论执行成功或参数错误，都删除队首命令；
         * 否则一个错误命令会在无线链路上无限重发。 */
        if (ack.command == APP_CMD_SET_RELAY ||
            ack.command == APP_CMD_GET_RELAY_STATE) {
            if (ack.result == APP_RESULT_OK) {
                publish_relay_state(active_node, &ack);
            }
        }
        remove_first_command();

        radio_state = GATEWAY_RADIO_IDLE;
        next_radio_action = (ack.result == APP_RESULT_OK) ? now
                                                          : now + GATEWAY_COMMAND_RETRY_MS;

    /* ---------- 遥测数据 ---------- */
    } else if (active_expected_type == APP_MSG_TELEMETRY &&
               header->payload_len == sizeof(AppTelemetry)) {

        AppTelemetry telemetry;
        memcpy(&telemetry, payload, sizeof(telemetry));

        publish_telemetry(active_node, &telemetry);
        schedule_next_poll(active_node, now);

        radio_state = GATEWAY_RADIO_IDLE;
        next_radio_action = now;
    }
}

/* ============================================================
 * 公共接口
 * ============================================================ */

/**
 * @brief 初始化网关应用。
 *
 * 清零所有队列和状态，初始化节点轮询时间，并打印 MQTT 解析器版本。
 */
void gateway_init(void)
{
    uint8_t index;
    char    parser_line[80];

    memset(command_queue, 0, sizeof(command_queue));
    memset(publish_queue, 0, sizeof(publish_queue));

    command_count        = 0u;
    publish_head         = 0u;
    publish_count        = 0u;
    active_node          = 0u;
    active_expected_type = 0u;
    active_command_id    = 0u;
    next_sequence        = 0u;
    radio_state          = GATEWAY_RADIO_IDLE;
    next_radio_action    = board_millis();

    for (index = 0u; index < GATEWAY_NODE_COUNT; ++index) {
        nodes[index].id        = gateway_node_ids[index];
        nodes[index].next_poll = board_millis() + GATEWAY_NORMAL_POLL_MS;
    }

    /* 打印 MQTT 命令解析器版本，便于确认固件是否更新 */
    (void)snprintf(parser_line, sizeof(parser_line), "MQTT PARSER=%s\r\n",
                   mqtt_command_parser_version());
    gateway_debug(parser_line);
}

/**
 * @brief MQTT 下行消息回调：解析命令并加入命令队列。
 *
 * 解析失败时打印详细诊断信息（解析器版本、主题长度、载荷长度、
 * 首尾字节），方便定位 JSON 格式问题。
 */
void gateway_on_mqtt_message(const char *topic, const char *payload)
{
    MqttCommand       command;
    MqttCommandResult result;

    if (!topic || !payload) {
        gateway_debug("MQTT CMD DROP: null\r\n");
        return;
    }

    result = mqtt_command_parse(topic, payload, gateway_node_ids,
                                GATEWAY_NODE_COUNT, &command);

    if (result != MQTT_COMMAND_OK) {
        char   text[192];
        size_t topic_length   = strlen(topic);
        size_t payload_length = strlen(payload);
        unsigned first = payload_length ? (unsigned)(uint8_t)payload[0] : 0u;
        unsigned last  = payload_length ?
                         (unsigned)(uint8_t)payload[payload_length - 1u] : 0u;

        (void)snprintf(text, sizeof(text),
                       "MQTT CMD DROP parse=%u detail=%s topic_len=%lu "
                       "payload_len=%lu first=%02X last=%02X\r\n",
                       (unsigned)result, mqtt_command_last_detail(),
                       (unsigned long)topic_length,
                       (unsigned long)payload_length, first, last);
        gateway_debug(text);
        return;
    }

    if (command_count >= GATEWAY_COMMAND_QUEUE_SIZE) {
        gateway_debug("MQTT CMD DROP queue full\r\n");
        return;
    }

    command_queue[command_count].command = command;
    command_queue[command_count].used    = 1u;
    ++command_count;

    /* 命令到达后立即触发无线任务，缩短响应延迟 */
    next_radio_action = board_millis();

    {
        char text[128];
        (void)snprintf(text, sizeof(text),
                       "MQTT CMD QUEUED node=%u id=%lu code=%u value=%lu\r\n",
                       command.node, (unsigned long)command.id, command.code,
                       (unsigned long)command.value);
        gateway_debug(text);
    }
}

/**
 * @brief 无线通信任务（需在主循环中反复调用）。
 *
 * 状态机：
 *   IDLE —— 检查是否有待发命令或到期轮询，构造帧并发送后进入 WAIT
 *   WAIT —— 轮询接收从机响应，验证并处理；超时则回到 IDLE
 */
void gateway_radio_task(void)
{
    uint8_t  address[5];
    uint8_t  rx[APP_RADIO_MAX_FRAME];
    uint8_t  rx_length;
    uint8_t  hw_ack;
    uint8_t  received;
    const AppHeader *header;
    const uint8_t   *payload;
    uint32_t now = board_millis();

    /* ---------- 等待从机响应 ---------- */
    if (radio_state == GATEWAY_RADIO_WAIT) {
        received = nrf24_receive(rx, &rx_length);

        if (received) {
            if (!app_validate_frame(rx, rx_length, &header, &payload)) {
                gateway_debug("RF RX DROP format\r\n");
            } else if (header->src_id != active_node ||
                       header->dst_id != APP_MASTER_ID) {
                /* 源地址或目标地址不匹配，丢弃 */
                char text[96];
                (void)snprintf(text, sizeof(text),
                               "RF RX DROP id src=%u dst=%u active=%u\r\n",
                               header->src_id, header->dst_id, active_node);
                gateway_debug(text);
            } else if (header->type != active_expected_type) {
                /* 消息类型不符，丢弃 */
                char text[80];
                (void)snprintf(text, sizeof(text),
                               "RF RX DROP type=%u expected=%u\r\n",
                               header->type, active_expected_type);
                gateway_debug(text);
            } else {
                finish_response(header, payload, now);
                return;
            }
        }

        /* 应用层超时：打印诊断信息并回到空闲 */
        if (due(now, application_deadline)) {
            char text[128];
            (void)snprintf(text, sizeof(text),
                           "RF APP TIMEOUT node=%u expected=%u status=0x%02X fifo=0x%02X\r\n",
                           active_node, active_expected_type,
                           nrf24_read_register(0x07u),
                           nrf24_read_register(0x17u));
            gateway_debug(text);
            radio_state = GATEWAY_RADIO_IDLE;
            next_radio_action = now + GATEWAY_COMMAND_RETRY_MS;
        }
        return;
    }

    /* ---------- 空闲状态：未到时间直接返回 ---------- */
    if (!due(now, next_radio_action)) {
        return;
    }

    /* 优先发送命令，其次轮询到期节点 */
    if (command_count) {
        start_command(&command_queue[0].command);
    } else if (select_due_node(now, &active_node)) {
        start_poll(active_node);
        /* 无论本次无线收发是否成功，都按正常周期安排下次轮询，
         * 避免失联节点被 100ms 连续轰炸。 */
        schedule_next_poll(active_node, now);
    } else {
        return;
    }

    node_address(active_node, address);

    /*
     * TX_DS 只代表主机收到了 nRF 硬件 ACK。该 ACK 可能单独丢失，而从机
     * 实际已收到并执行了请求。因此即使 nrf24_send 返回失败，主机也必须
     * 立即切回 PRX 等待应用响应，不能留在 PTX 后直接安排下一次重发。
     */
    hw_ack = nrf24_send(address, tx_frame, APP_RADIO_MAX_FRAME, 30u);
    nrf24_start_listening(MASTER_ADDRESS);

    if (!hw_ack) {
        gateway_debug("RF NO_HW_ACK; PRX WAIT\r\n");
    }

    /* nrf24_send 本身可阻塞数十毫秒，应用等待窗口从实际进入 PRX 后计时。 */
    application_deadline = board_millis() + GATEWAY_APPLICATION_TIMEOUT_MS;
    radio_state = GATEWAY_RADIO_WAIT;
}

/**
 * @brief MQTT 发布任务（需在主循环中反复调用）。
 *
 * 将发布队列中的消息依次交给 CLM970 驱动发送。
 * 单条发送失败则整体退出，避免阻塞主循环。
 */
void gateway_publish_task(void)
{
    GatewayPublishItem *item;

    while (publish_count) {
        item = &publish_queue[publish_head];

        if (!item->used ||
            !clm970_publish(item->topic, item->payload, 1u, item->retained)) {
            return;
        }

        item->used = 0u;
        publish_head = (uint8_t)((publish_head + 1u) % GATEWAY_PUBLISH_QUEUE_SIZE);
        --publish_count;
    }
}