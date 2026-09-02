#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

#define DBG(...)                             \
    do                                       \
    {                                        \
        if (DEBUG_ENABLED)                   \
        {                                    \
            Serial.printf(__VA_ARGS__);      \
        }                                    \
    } while (0)

// 常量定义
static const uint16_t MAGIC = 0x4E52;
static const uint16_t NETWORK_ID = 0x2026;
static const uint16_t MASTER_ID = 0;

// 消息类型
enum
{
    MSG_POLL = 1,
    MSG_TELEMETRY = 2,
    MSG_COMMAND = 0x10,
    MSG_COMMAND_ACK = 0x11
};

// 命令码
enum
{
    CMD_SET_LED = 1,
    CMD_SET_PERIOD = 2,
    CMD_FORCE_SAMPLE = 3,
    CMD_DEBUG_DUMP = 4
};

// 节点状态
enum NodeState : uint8_t
{
    NODE_UNKNOWN,
    NODE_ONLINE,
    NODE_DEGRADED,
    NODE_OFFLINE,
    NODE_RECOVERING
};

// 无线状态机
enum RadioState : uint8_t
{
    RADIO_IDLE,
    RADIO_SEND,
    RADIO_WAIT
};

#pragma pack(push, 1)
struct Header
{
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t network;
    uint16_t src;
    uint16_t dst;
    uint16_t boot;
    uint16_t seq;
    uint8_t length;
    uint8_t flags;
};

struct Telemetry
{
    uint16_t requestSeq;
    int16_t temp;
    uint16_t humidity;
    uint32_t pressure;
    uint16_t age;
    uint8_t status;
    uint8_t led;
};

struct Command
{
    uint32_t id;
    uint8_t code;
    uint8_t reserved;
    uint32_t value;
};

struct CommandAck
{
    uint32_t id;
    uint8_t code;
    uint8_t result;
    uint32_t value;
};
#pragma pack(pop)

struct NodeRuntime
{
    uint8_t id;
    NodeState state;
    uint8_t failures;
    uint8_t successes;
    uint32_t lastSeen;
    uint32_t lastData;
    uint32_t nextPoll;
    uint16_t boot;
    uint16_t lastSeq;
    bool hasSeq;
    Telemetry data;
};

struct QueuedCommand
{
    uint8_t node;
    uint8_t code;
    uint32_t id;
    uint32_t value;
};

struct PublishItem
{
    char topic[96];
    char payload[MQTT_BUFFER_SIZE];
    bool retained;
};

// 全局对象定义
RF24 radio(D1, D2);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

const uint8_t MASTER_ADDR[5] = {0xD2, 0xD2, 0xD2, 0xD2, 0xA0};

// 全局状态变量
NodeRuntime nodes[NODE_COUNT];
QueuedCommand cmdQueue[8];
uint8_t cmdHead = 0, cmdTail = 0, cmdCount = 0;

PublishItem pubQueue[12];
uint8_t pubHead = 0, pubTail = 0, pubCount = 0;

RadioState radioState = RADIO_IDLE;
uint8_t activeNode = 0;
uint8_t request[32];
uint8_t expectedType;
uint8_t attempts;
uint16_t activeSeq = 0;
uint16_t nextSeq = 0;
uint16_t masterBoot;
uint32_t activeCommandId = 0;
uint32_t responseDeadline;
uint32_t retryAt;
uint32_t nextWifiAttempt;
uint32_t nextMqttAttempt;
uint8_t activeCommandCode = 0;
uint32_t activeCommandValue = 0;
bool activeWasCommand = false;
bool oledReady = false;

uint32_t lastHealth;
uint32_t lastOled;
uint8_t oledPage = 0;

/**
 * 计算与STM32完全相同的CRC-16/CCITT-FALSE
 * 覆盖逻辑帧头和有效载荷
 */
static uint16_t crc16(const uint8_t *d, uint8_t n)
{
    uint16_t c = 0xFFFF;
    while (n--)
    {
        c ^= (uint16_t)*d++ << 8;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (c & 0x8000)
            {
                c = (uint16_t)((c << 1) ^ 0x1021);
            }
            else
            {
                c = (uint16_t)(c << 1);
            }
        }
    }
    return c;
}

/**
 * 根据节点ID生成唯一5字节接收地址
 * 前4字节固定，末字节区分节点
 */
static void nodeAddress(uint8_t id, uint8_t a[5])
{
    a[0] = a[1] = a[2] = a[3] = 0xE7;
    a[4] = id;
}

/**
 * 构造主机发往指定节点的固定32字节应用帧
 * 参数p可为空；n最大14字节
 * 返回0表示载荷超限，成功返回32
 */
static uint8_t buildFrame(uint8_t *out, uint8_t type, uint8_t node, uint16_t seq, const void *p, uint8_t n)
{
    if (n > 14)
    {
        return 0;
    }

    memset(out, 0, 32);
    Header h = {
        MAGIC,
        1,
        type,
        NETWORK_ID,
        MASTER_ID,
        node,
        masterBoot,
        seq,
        n,
        0
    };
    memcpy(out, &h, sizeof(h));

    if (n)
    {
        memcpy(out + sizeof(h), p, n);
    }

    uint8_t logical = sizeof(h) + n;
    uint16_t c = crc16(out, logical);
    out[logical] = c;
    out[logical + 1] = c >> 8;
    return 32;
}

/**
 * 校验从机响应的魔数、协议版本、网络号、长度和应用CRC
 * 成功时h和p指向原32字节缓冲区内部，不会复制数据
 */
static bool validate(const uint8_t *f, const Header *&h, const uint8_t *&p)
{
    h = (const Header *)f;
    if (h->magic != MAGIC || h->version != 1 || h->network != NETWORK_ID || h->length > 14)
    {
        return false;
    }

    uint8_t n = sizeof(Header) + h->length + 2;
    if (n > 32)
    {
        return false;
    }

    uint16_t got = f[n - 2] | ((uint16_t)f[n - 1] << 8);
    if (got != crc16(f, n - 2))
    {
        return false;
    }

    p = f + sizeof(Header);
    return true;
}

/**
 * 将节点状态枚举转换为串口、MQTT和OLED使用的稳定英文名称
 */
static const char *stateName(NodeState s)
{
    static const char *n[] = {"unknown", "online", "degraded", "offline", "recovering"};
    return n[s];
}

/**
 * 按节点ID查找运行时记录
 * 找不到返回nullptr，用于拒绝未知节点命令
 */
static NodeRuntime *findNode(uint8_t id)
{
    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        if (nodes[i].id == id)
        {
            return &nodes[i];
        }
    }
    return nullptr;
}

// 前向声明
static void publishCommandResult(uint8_t node, const CommandAck &a);

/**
 * 将MQTT消息复制进固定长度发布环形队列
 * 网络断线时仍可暂存
 * 返回false表示队列已满；函数不会阻塞等待MQTT服务器
 */
static bool enqueuePublish(const char *topic, const char *payload, bool retained = false)
{
    if (pubCount == 12)
    {
        DBG("PUB DROP full topic=%s\n", topic);
        return false;
    }

    PublishItem &i = pubQueue[pubTail];
    strlcpy(i.topic, topic, sizeof(i.topic));
    strlcpy(i.payload, payload, sizeof(i.payload));
    i.retained = retained;
    pubTail = (pubTail + 1) % 12;
    pubCount++;
    return true;
}

/**
 * 把节点在线状态序列化为JSON并放入MQTT发布队列
 * 状态主题使用Retain
 */
static void publishState(NodeRuntime &n)
{
    char topic[96];
    char json[192];

    snprintf(topic, sizeof(topic), "environment/%s/node/%u/state", GATEWAY_ID, n.id);
    snprintf(json, sizeof(json), "{\"node_id\":%u,\"state\":\"%s\",\"last_seen_ms\":%lu,\"failures\":%u}",
             n.id, stateName(n.state), n.lastSeen, n.failures);
    enqueuePublish(topic, json, true);
}

/**
 * 仅在状态发生变化时更新、打印并发布
 * 避免重复MQTT状态消息
 */
static void setState(NodeRuntime &n, NodeState s)
{
    if (n.state == s)
    {
        return;
    }

    DBG("NODE %u %s -> %s\n", n.id, stateName(n.state), stateName(s));
    n.state = s;
    publishState(n);
}

/**
 * 记录一次完整应用事务成功
 * 清失败计数、更新时间，并推进恢复状态机
 */
static void markSuccess(NodeRuntime &n)
{
    n.lastSeen = millis();
    n.failures = 0;
    n.successes++;

    if (n.state == NODE_UNKNOWN || n.state == NODE_OFFLINE)
    {
        setState(n, NODE_RECOVERING);
    }

    if ((n.state == NODE_RECOVERING && n.successes >= ONLINE_RECOVERY_OK) || n.state == NODE_DEGRADED)
    {
        setState(n, NODE_ONLINE);
    }

    n.nextPoll = millis() + NORMAL_POLL_MS;
}

/**
 * 记录一次事务失败
 * 并按连续失败次数切换degraded/offline及降低探测频率
 */
static void markFailure(NodeRuntime &n)
{
    n.successes = 0;
    if (n.failures < 255)
    {
        n.failures++;
    }

    if (n.failures >= OFFLINE_FAILURES)
    {
        setState(n, NODE_OFFLINE);
    }
    else if (n.failures >= DEGRADED_FAILURES)
    {
        setState(n, NODE_DEGRADED);
    }

    n.nextPoll = millis() + (n.state == NODE_OFFLINE ? OFFLINE_PROBE_MS : NORMAL_POLL_MS);
}

/**
 * 将MQTT或串口命令加入固定8项下行队列
 * 返回false表示目标节点未配置或队列已满
 */
static bool enqueueCommand(uint8_t node, uint8_t code, uint32_t value, uint32_t id)
{
    if (!findNode(node) || cmdCount == 8)
    {
        return false;
    }

    cmdQueue[cmdTail] = {node, code, id, value};
    cmdTail = (cmdTail + 1) % 8;
    cmdCount++;
    return true;
}

/**
 * 如果队头命令属于当前节点，则弹出并复制到out
 * 保持命令FIFO顺序
 */
static bool takeCommandFor(uint8_t node, QueuedCommand &out)
{
    if (!cmdCount || cmdQueue[cmdHead].node != node)
    {
        return false;
    }

    out = cmdQueue[cmdHead];
    cmdHead = (cmdHead + 1) % 8;
    cmdCount--;
    return true;
}

/**
 * 为节点启动一次逻辑事务
 * 命令优先，否则构造普通POLL
 * 此函数只准备帧和状态，真正SPI发送由radioTask下一次调度完成
 */
static void beginRequest(uint8_t index)
{
    NodeRuntime &n = nodes[index];
    QueuedCommand c;

    activeNode = index;
    activeSeq = ++nextSeq;
    attempts = 0;
    activeCommandId = 0;
    activeWasCommand = takeCommandFor(n.id, c);

    if (activeWasCommand)
    {
        Command p = {c.id, c.code, 0, c.value};
        activeCommandId = c.id;
        activeCommandCode = c.code;
        activeCommandValue = c.value;
        buildFrame(request, MSG_COMMAND, n.id, activeSeq, &p, sizeof(p));
        expectedType = MSG_COMMAND_ACK;
    }
    else
    {
        buildFrame(request, MSG_POLL, n.id, activeSeq, nullptr, 0);
        expectedType = MSG_TELEMETRY;
    }

    radioState = RADIO_SEND;
    retryAt = millis();
}

/**
 * 结束当前无线事务并更新节点健康度
 * 命令最终超时时发布result=0xFE，让云端知道命令没有完成
 */
static void finishRequest(bool ok, const char *reason)
{
    NodeRuntime &n = nodes[activeNode];

    if (ok)
    {
        markSuccess(n);
    }
    else
    {
        DBG("TIMEOUT[%s] node=%u seq=%u\n", reason, n.id, activeSeq);
        markFailure(n);

        if (activeWasCommand)
        {
            CommandAck a = {activeCommandId, activeCommandCode, 0xFE, activeCommandValue};
            publishCommandResult(n.id, a);
        }
    }

    radioState = RADIO_IDLE;
}

/**
 * 按boot_id+16位seq判断遥测是否为新消息，并正确处理序号回绕
 * 重复遥测仍可让事务成功，但不会重复发布到MQTT
 */
static bool seqNew(NodeRuntime &n, uint16_t boot, uint16_t seq)
{
    if (!n.hasSeq || n.boot != boot)
    {
        n.boot = boot;
        n.lastSeq = seq;
        n.hasSeq = true;
        return true;
    }

    int16_t d = (int16_t)(seq - n.lastSeq);
    if (d > 0)
    {
        n.lastSeq = seq;
        return true;
    }

    return false;
}

/**
 * 将定点遥测转换为JSON并加入发布队列
 * 无线层不传输JSON和浮点字符串
 */
static void publishTelemetry(NodeRuntime &n, const Header *h, const Telemetry &t)
{
    char topic[96];
    char json[MQTT_BUFFER_SIZE];

    snprintf(topic, sizeof(topic), "environment/%s/node/%u/telemetry", GATEWAY_ID, n.id);
    snprintf(json, sizeof(json), "{\"node_id\":%u,\"boot_id\":%u,\"seq\":%u,\"temperature\":%.2f,\"humidity\":%.2f,\"pressure_pa\":%lu,\"sample_age_ms\":%u,\"sensor_status\":%u,\"led\":%u}",
             n.id, h->boot, h->seq, t.temp / 100.0f, t.humidity / 100.0f, t.pressure, t.age, t.status, t.led);
    enqueuePublish(topic, json, false);
}

/**
 * 将从机命令回执或网关超时结果转换为JSON并排队发布
 */
static void publishCommandResult(uint8_t node, const CommandAck &a)
{
    char topic[96];
    char json[192];

    snprintf(topic, sizeof(topic), "environment/%s/node/%u/command/result", GATEWAY_ID, node);
    snprintf(json, sizeof(json), "{\"node_id\":%u,\"command_id\":%lu,\"code\":%u,\"result\":%u,\"value\":%lu}",
             node, a.id, a.code, a.result, a.value);
    enqueuePublish(topic, json, false);
}

/**
 * 网关无线非阻塞状态机：
 * IDLE选节点 -> SEND发送/硬件ACK -> WAIT应用响应
 * 硬件ACK失败打印NO_HW_ACK；有ACK但100ms无合法响应打印NO_APP_RESPONSE
 * 每次loop只推进有限步骤，避免影响Wi-Fi、MQTT和看门狗喂狗
 */
static void radioTask()
{
    // 空闲状态：发送命令或轮询节点
    if (radioState == RADIO_IDLE)
    {
        if (cmdCount)
        {
            NodeRuntime *n = findNode(cmdQueue[cmdHead].node);
            if (n)
            {
                beginRequest((uint8_t)(n - nodes));
                return;
            }
            cmdHead = (cmdHead + 1) % 8;
            cmdCount--;
        }

        for (uint8_t i = 0; i < NODE_COUNT; i++)
        {
            if ((int32_t)(millis() - nodes[i].nextPoll) >= 0)
            {
                beginRequest(i);
                break;
            }
        }
        return;
    }

    NodeRuntime &n = nodes[activeNode];

    // 发送状态
    if (radioState == RADIO_SEND)
    {
        if ((int32_t)(millis() - retryAt) < 0)
        {
            return;
        }

        uint8_t a[5];
        nodeAddress(n.id, a);
        attempts++;

        radio.stopListening();
        radio.openWritingPipe(a);
        bool delivered = radio.write(request, 32);
        radio.openReadingPipe(1, MASTER_ADDR);
        radio.startListening();

        if (!delivered)
        {
            DBG("RF NO_HW_ACK node=%u try=%u\n", n.id, attempts);

            if (attempts < 3)
            {
                retryAt = millis() + 5;
                return;
            }

            finishRequest(false, "NO_HW_ACK");
            return;
        }

        responseDeadline = millis() + 100;
        radioState = RADIO_WAIT;
    }

    // 等待响应状态
    if (radioState == RADIO_WAIT)
    {
        while (radio.available())
        {
            uint8_t f[32];
            radio.read(f, 32);

            const Header *h;
            const uint8_t *p;

            if (!validate(f, h, p))
            {
                DBG("RF DROP CRC\n");
                continue;
            }

            if (h->src != n.id || h->dst != MASTER_ID || h->type != expectedType)
            {
                continue;
            }

            // 处理遥测数据
            if (h->type == MSG_TELEMETRY && h->length == sizeof(Telemetry))
            {
                Telemetry t;
                memcpy(&t, p, sizeof(t));

                if (t.requestSeq != activeSeq)
                {
                    continue;
                }

                bool fresh = seqNew(n, h->boot, h->seq);
                n.data = t;
                n.lastData = millis();

                if (fresh)
                {
                    publishTelemetry(n, h, t);
                }

                DBG("DATA n=%u T=%d H=%u P=%lu st=%02X\n", n.id, t.temp, t.humidity, t.pressure, t.status);
                finishRequest(true, nullptr);
                return;
            }

            // 处理命令应答
            if (h->type == MSG_COMMAND_ACK && h->length == sizeof(CommandAck))
            {
                CommandAck a;
                memcpy(&a, p, sizeof(a));

                if (a.id != activeCommandId)
                {
                    continue;
                }

                publishCommandResult(n.id, a);
                DBG("CMD ACK n=%u id=%lu result=%u\n", n.id, a.id, a.result);
                finishRequest(true, nullptr);
                return;
            }
        }

        // 超时处理
        if ((int32_t)(millis() - responseDeadline) >= 0)
        {
            DBG("RF NO_APP_RESPONSE node=%u try=%u\n", n.id, attempts);

            if (attempts < 3)
            {
                radioState = RADIO_SEND;
                retryAt = millis() + 5;
            }
            else
            {
                finishRequest(false, "NO_APP_RESPONSE");
            }
        }
    }
}

/**
 * 从简单MQTT JSON中提取一个无符号整数
 * 缺失或格式错误返回fallback
 * 当前命令协议仅含数字字段，若以后加入字符串/嵌套对象应换ArduinoJson
 */
static uint32_t jsonNumber(const String &s, const char *key, uint32_t fallback)
{
    String k = String('"') + key + '"';
    int p = s.indexOf(k);
    if (p < 0)
    {
        return fallback;
    }

    p = s.indexOf(':', p);
    if (p < 0)
    {
        return fallback;
    }

    return strtoul(s.c_str() + p + 1, nullptr, 10);
}

/**
 * PubSubClient收到下行命令时调用
 * 限制长度、解析数字字段并加入命令队列
 * 重要：回调中不直接发nRF，避免阻塞MQTT心跳和重入无线状态机
 */
static void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    if (length >= 384)
    {
        DBG("MQTT command too long\n");
        return;
    }

    char b[385];
    memcpy(b, payload, length);
    b[length] = 0;
    String s(b);

    uint8_t node = (uint8_t)jsonNumber(s, "node_id", 0);
    uint8_t code = (uint8_t)jsonNumber(s, "code", 0);
    uint32_t id = jsonNumber(s, "command_id", 0);
    uint32_t value = jsonNumber(s, "value", 0);

    if (!node || !code || !id || !enqueueCommand(node, code, value, id))
    {
        DBG("MQTT CMD rejected topic=%s\n", topic);
        return;
    }

    DBG("MQTT CMD queued n=%u id=%lu code=%u\n", node, id, code);
}

/**
 * 每10秒尝试一次Wi-Fi重连
 * 不使用阻塞while等待连接
 */
static void wifiTask()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return;
    }

    if ((int32_t)(millis() - nextWifiAttempt) < 0)
    {
        return;
    }

    nextWifiAttempt = millis() + 10000;
    DBG("WIFI reconnect\n");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

/**
 * 维护MQTT连接、遗嘱、订阅和发布队列
 * 每次只尝试发布队头一条，成功才出队
 * 断网期间无线轮询仍继续
 */
static void mqttTask()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return;
    }

    if (!mqtt.connected() && (int32_t)(millis() - nextMqttAttempt) >= 0)
    {
        nextMqttAttempt = millis() + 5000;
        String cid = String(GATEWAY_ID) + "-" + String(ESP.getChipId(), HEX);
        String will = String("environment/") + GATEWAY_ID + "/gateway/state";

        if (mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASSWORD, will.c_str(), 1, true, "offline"))
        {
            DBG("MQTT connected\n");
            String sub = String("environment/") + GATEWAY_ID + "/+/command/set";
            mqtt.subscribe(sub.c_str(), 1);
            mqtt.publish(will.c_str(), "online", true);
        }
    }

    if (!mqtt.connected())
    {
        return;
    }

    mqtt.loop();

    if (pubCount)
    {
        PublishItem &i = pubQueue[pubHead];
        if (mqtt.publish(i.topic, i.payload, i.retained))
        {
            pubHead = (pubHead + 1) % 12;
            pubCount--;
        }
    }
}

/**
 * 每秒检查节点最后成功时间
 * 处理没有及时被事务失败计数覆盖的失联情况
 */
static void healthTask()
{
    if (millis() - lastHealth < 1000)
    {
        return;
    }

    lastHealth = millis();

    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        NodeRuntime &n = nodes[i];

        if (n.state != NODE_UNKNOWN && n.state != NODE_OFFLINE && n.lastSeen &&
            millis() - n.lastSeen > OFFLINE_PROBE_MS)
        {
            setState(n, NODE_OFFLINE);
        }
    }
}

/**
 * 每秒刷新OLED一页
 * 显示网关联网状态、队列深度和轮播节点遥测
 */
static void oledTask()
{
    if (!oledReady || millis() - lastOled < 1000)
    {
        return;
    }

    lastOled = millis();
    uint8_t online = 0;

    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        if (nodes[i].state == NODE_ONLINE || nodes[i].state == NODE_RECOVERING)
        {
            online++;
        }
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.printf("GW %s M:%s", WiFi.status() == WL_CONNECTED ? "WiFi" : "----", mqtt.connected() ? "OK" : "--");
    
    display.setCursor(0, 10);
    display.printf("Nodes %u/%u Q:%u", online, (unsigned)NODE_COUNT, pubCount);

    NodeRuntime &n = nodes[oledPage % NODE_COUNT];
    display.setCursor(0, 24);
    display.printf("Node %u %s", n.id, stateName(n.state));

    display.setCursor(0, 36);
    display.printf("T %.2fC H %.2f%%", n.data.temp / 100.0f, n.data.humidity / 100.0f);

    display.setCursor(0, 48);
    display.printf("P %.2fhPa S:%02X", n.data.pressure / 100.0f, n.data.status);

    display.display();
    oledPage = (oledPage + 1) % NODE_COUNT;
}

/**
 * 非阻塞读取串口整行命令：status或cmd <node> <code> <value>
 * 使用固定64字节缓冲，不调用readStringUntil，避免默认1秒超时阻塞loop
 */
static void serialTask()
{
    static char b[64];
    static uint8_t n = 0;

    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\r')
        {
            continue;
        }

        if (c != '\n')
        {
            if (n < 63)
            {
                b[n++] = c;
            }
            continue;
        }

        b[n] = 0;
        unsigned int node, code;
        unsigned long value;

        if (sscanf(b, "cmd %u %u %lu", &node, &code, &value) == 3)
        {
            enqueueCommand((uint8_t)node, (uint8_t)code, value, millis());
        }
        else if (!strcmp(b, "status"))
        {
            for (uint8_t i = 0; i < NODE_COUNT; i++)
            {
                DBG("NODE %u %s fail=%u last=%lu\n", nodes[i].id, stateName(nodes[i].state), nodes[i].failures, nodes[i].lastSeen);
            }
        }

        n = 0;
    }
}

/**
 * 初始化节点表、nRF、OLED、Wi-Fi和MQTT
 * nRF SPI失败时停止以保留明确故障
 */
void setup()
{
    Serial.begin(115200);
    delay(50);

    masterBoot = (uint16_t)(ESP.getChipId() ^ micros());

    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        nodes[i] = NodeRuntime{};
        nodes[i].id = CONFIGURED_NODE_IDS[i];
        nodes[i].state = NODE_UNKNOWN;
        nodes[i].nextPoll = 500 + i * 200;
    }

    if (!radio.begin() || !radio.isChipConnected())
    {
        Serial.println("FATAL NRF SPI");
        while (true)
        {
            delay(1000);
        }
    }

    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MIN);
    radio.setAddressWidth(5);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(true);
    radio.setRetries(3, 5);
    radio.disableDynamicPayloads();
    radio.setPayloadSize(32);
    radio.openReadingPipe(1, MASTER_ADDR);
    radio.startListening();
    radio.printDetails();

    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    if (OLED_ENABLED)
    {
        oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
        if (!oledReady)
        {
            Serial.println("OLED init failed");
        }
    }

    WiFi.persistent(false);
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(MQTT_BUFFER_SIZE);

    wifiTask();
    DBG("GATEWAY BOOT nodes=%u\n", (unsigned)NODE_COUNT);
}

/**
 * 主事件循环
 * 各任务都应快速返回，yield用于维持ESP8266 Wi-Fi栈和软件看门狗
 */
void loop()
{
    radioTask();
    wifiTask();
    mqttTask();
    healthTask();
    oledTask();
    serialTask();
    yield();
}
