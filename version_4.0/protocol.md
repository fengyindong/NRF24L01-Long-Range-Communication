一、协议总体设计
1. 设计目标
目标	实现方式
定长传输	所有帧固定 32 字节，避免 nRF24 动态载荷兼容问题
可靠校验	硬件 CRC + 应用层 CRC-16 双层保护
防止串扰	魔数 + 网络 ID + 源/目标节点 ID 四重过滤
通用命令	帧头统一，命令码区分类型
去重能力	boot_id + seq 请求去重，command_id 命令去重
紧凑高效	位运算编码继电器命令，一个 32 位整数表达多路控制
2. 帧的物理结构
text
┌─────────────────────────────────────────────────────────────┐
│                   32 字节定长帧                              │
├────────────────────────────┬──────────────────┬─────────────┤
│      帧头 (16 字节)         │  载荷 (0~14 字节) │ CRC (2 字节)│
├────────────────────────────┼──────────────────┼─────────────┤
│  magic/version/type/...    │  业务数据         │ 低字节 高字节│
└────────────────────────────┴──────────────────┴─────────────┘
                              ← 不足部分补 0 →
二、协议常量
宏	值	含义
APP_MAGIC	0x4E52u	帧起始魔数（"NR" 的 ASCII）
APP_VERSION	1u	协议版本号
APP_NETWORK_ID	0x2026u	网络 ID，防止不同网络间串扰
APP_MASTER_ID	0u	主机节点 ID，固定为 0
APP_THIS_NODE_ID	APP_NODE_ID	本节点 ID，取自 config.h
APP_MAX_PAYLOAD	14u	最大载荷长度
APP_HEADER_SIZE	16u	帧头长度
APP_MAX_FRAME_SIZE	32u	最大帧长
APP_RADIO_MAX_FRAME	32u	射频层固定帧长
关键约束：

16 (帧头) + 14 (载荷) + 2 (CRC) = 32 —— 刚好填满 nRF24 的固定载荷。

载荷超过 14 字节会直接拒绝组帧。

三、消息类型（type 字段）
值	名称	方向	用途
0x01	APP_MSG_POLL	主机 → 从机	轮询遥测，载荷为空
0x02	APP_MSG_TELEMETRY	从机 → 主机	上报遥测数据
0x10	APP_MSG_COMMAND	主机 → 从机	下发控制命令
0x11	APP_MSG_COMMAND_ACK	从机 → 主机	回复命令执行结果
四、命令码（command 字段）
值	名称	参数（value）	说明
0x01	APP_CMD_SET_LED	0=灭，1=亮	设置 PC13 LED
0x02	APP_CMD_SET_SAMPLE_PERIOD	周期毫秒（1000~60000）	设置采样周期
0x03	APP_CMD_FORCE_SAMPLE	无	强制立即采样一次
0x04	APP_CMD_DEBUG_DUMP	无	输出 nRF 寄存器和传感器状态
0x10	APP_CMD_SET_RELAY	位域编码（见下）	设置单路继电器
0x11	APP_CMD_GET_RELAY_STATE	无	查询继电器状态
继电器命令的位域编码
text
value (32 位):
  bit0~bit3 : 通道掩码（必须为 2 的幂，即只选中一路）
              bit0=继电器1, bit1=继电器2, bit2=继电器3, bit3=继电器4
  bit8~bit11: 目标状态（对应位为 1 吸合，为 0 断开）
  其他位    : 必须为 0
举例：控制继电器 3 吸合

text
channel = 3 → bit = 1 << 2 = 0x04
on      = 1 → bit8~11 对应位 = 0x04
value   = 0x04 | (0x04 << 8) = 0x0404
五、命令执行结果码（result 字段）
值	名称	含义
0	APP_RESULT_OK	执行成功
1	APP_RESULT_INVALID_PARAM	参数无效
2	APP_RESULT_UNSUPPORTED	不支持的命令
3	APP_RESULT_BAD_FRAME	帧格式或 CRC 错误
六、帧头结构体 AppHeader（16 字节）
c
#pragma pack(push, 1)
typedef struct {
    uint16_t magic;        /* 2B: 魔数 0x4E52 */
    uint8_t  version;      /* 1B: 版本号 1 */
    uint8_t  type;         /* 1B: 消息类型 */
    uint16_t network_id;   /* 2B: 网络 ID 0x2026 */
    uint16_t src_id;       /* 2B: 源节点 ID */
    uint16_t dst_id;       /* 2B: 目标节点 ID */
    uint16_t boot_id;      /* 2B: 节点启动序号 */
    uint16_t seq;          /* 2B: 帧序列号 */
    uint8_t  payload_len;  /* 1B: 载荷长度（0~14） */
    uint8_t  flags;        /* 1B: 标志位（预留，置 0） */
} AppHeader;
#pragma pack(pop)
字段详解：

字段	用途	主/从使用
magic	快速识别本协议帧	两端校验
version	协议兼容性检查	两端校验
type	消息类型分发	两端分发
network_id	防止不同网络串扰	两端校验
src_id	源节点	从机检查是否为主机；主机检查响应来源
dst_id	目标节点	从机检查是否为自己；主机检查是否为自己
boot_id	检测节点重启 + 请求去重	从机缓存，用于去重
seq	帧序列号 + 请求去重	从机缓存，用于去重；遥测中回带 request_seq
payload_len	载荷实际长度	用于计算 CRC 范围
flags	预留	当前置 0
七、业务载荷结构体
1. AppTelemetry（从机 → 主机，遥测）
c
typedef struct {
    uint16_t request_seq;         /* 2B: 对应主机的 POLL 序列号 */
    int16_t  temperature_centi_c; /* 2B: 温度 0.01℃ */
    uint16_t humidity_centi_rh;   /* 2B: 湿度 0.01%RH */
    uint32_t pressure_pa;         /* 4B: 气压 Pa */
    uint16_t sample_age_ms;       /* 2B: 数据年龄（毫秒） */
    uint8_t  sensor_status;       /* 1B: 传感器状态位 */
    uint8_t  led_state;           /* 1B: LED 状态 */
} AppTelemetry;   /* 共 14 字节，正好是最大载荷 */
用途：

主机收到后，通过 gateway_publish_task 转换为 JSON 发布到 MQTT。

sensor_status 是位标志，包含 AHT20/BMP280/数据新鲜/主机丢失等信息。

2. AppCommand（主机 → 从机，命令）
c
typedef struct {
    uint32_t command_id;  /* 4B: 命令唯一 ID，用于去重和应答匹配 */
    uint8_t  command;     /* 1B: 命令码 */
    uint8_t  reserved;    /* 1B: 保留，置 0 */
    uint32_t value;       /* 4B: 命令参数 */
} AppCommand;   /* 共 10 字节 */
3. AppCommandAck（从机 → 主机，命令应答）
c
typedef struct {
    uint32_t command_id;  /* 4B: 对应命令 ID */
    uint8_t  command;     /* 1B: 命令码 */
    uint8_t  result;      /* 1B: 执行结果 */
    uint32_t value;       /* 4B: 返回值或当前状态 */
} AppCommandAck;   /* 共 10 字节 */
八、传感器状态位（sensor_status）
位	名称	含义
bit0	SENSOR_AHT20_OK	AHT20 通信正常
bit1	SENSOR_BMP280_OK	BMP280 通信正常
bit2	SENSOR_AHT20_CRC_OK	AHT20 CRC 校验通过
bit3	SENSOR_DATA_FRESH	数据为最新采集
bit4	SENSOR_AHT20_ERROR	AHT20 异常
bit5	SENSOR_BMP280_ERROR	BMP280 异常
bit7	SENSOR_MASTER_LOST	从机检测到主机失联
注意：SENSOR_MASTER_LOST 是"主机丢失"，不是传感器故障。

九、函数详解
1. app_crc16() —— CRC-16/CCITT-FALSE
c
uint16_t app_crc16(const uint8_t *data, uint8_t length);
算法参数：

初始值：0xFFFF

多项式：0x1021（CCITT）

输入数据不反转

结果不异或

算法步骤：

text
crc = 0xFFFF
for each byte b:
    crc ^= (b << 8)
    for i in 0..7:
        if crc & 0x8000:
            crc = (crc << 1) ^ 0x1021
        else:
            crc = crc << 1
return crc
用途：在 nRF24 硬件 CRC 之外，再叠加一层应用层校验。硬件 CRC 只保护空中传输，应用层 CRC 还能发现软件组帧错误。

2. app_build_frame() —— 组帧
c
uint8_t app_build_frame(uint8_t *out, uint8_t type, uint16_t src_id,
                        uint16_t dst_id, uint16_t boot_id, uint16_t seq,
                        const void *payload, uint8_t payload_len);
流程：

参数校验：payload_len > 14 返回 0；总长 > 32 返回 0。

清空缓冲区：memset(out, 0, 32)，确保填充部分为 0。

填充帧头：按字段依次赋值。

拷贝载荷：memcpy(out + 16, payload, payload_len)。

计算 CRC：crc = app_crc16(out, 16 + payload_len)。

追加 CRC：低字节在前，高字节在后（小端序）。

返回 32。

注意：载荷可以为 NULL（比如 POLL 帧），此时 payload_len 必须为 0。

3. app_validate_frame() —— 校验
c
uint8_t app_validate_frame(const uint8_t *frame, uint8_t frame_len,
                           const AppHeader **header, const uint8_t **payload);
流程：

参数非空 + frame_len == 32。

强制类型转换为 AppHeader *。

帧头字段校验：

magic == APP_MAGIC

version == APP_VERSION

network_id == APP_NETWORK_ID

payload_len <= 14

expected = 16 + payload_len + 2 <= 32

CRC 校验：

从 frame[expected-2] 和 frame[expected-1] 拼出 got_crc。

app_crc16(frame, expected-2) 得到 calculated。

两者相等才通过。

输出指针：

*header = (AppHeader *)frame

*payload = frame + 16

生命周期：输出的 header 和 payload 指向 frame 内部，生命周期与 frame 相同。上层如果要把数据保存下来，必须立刻 memcpy。

4. app_relay_encode() —— 单路继电器编码
c
uint8_t app_relay_encode(uint8_t channel, uint8_t on, uint32_t *value);
参数校验：

channel 必须在 1~4 之间

on 必须为 0 或 1

编码：

c
bit = 1 << (channel - 1);
*value = bit | (on ? (bit << 8) : 0);
示例：

channel=1, on=1 → value = 0x0001 | 0x0100 = 0x0101

channel=4, on=0 → value = 0x0008 | 0 = 0x0008

5. app_relay_apply() —— 单路继电器解码
c
uint8_t app_relay_apply(uint8_t current_state, uint32_t value,
                        uint8_t *next_state);
参数校验：

value 只能使用低 4 位和 bit8~11，其他位必须为 0。

mask 不能为 0。

mask 必须是 2 的幂（单路）：mask & (mask - 1) == 0。

requested 必须是 mask 的子集。

解码：

c
mask      = value & 0x0F;
requested = (value >> 8) & 0x0F;
*next_state = (current_state & ~mask) | (requested & mask);
位运算意义：

current_state & ~mask：选中的位清零，保留其他位。

requested & mask：只保留选中的位。

按位或：选中的位用新值，未选中的位保持原状。

示例：

当前 0b1010（继电器 2、4 吸合）

命令 value = 0x0404（控制继电器 3 吸合）

mask = 0x04，requested = 0x04

next_state = (0b1010 & 0b1011) | (0b0100 & 0b0100) = 0b1010 | 0b0100 = 0b1110

十、编译期断言
c
typedef char assert_header_size [(sizeof(AppHeader) == 16u) ? 1 : -1];
typedef char assert_command_size[(sizeof(AppCommand) == 10u) ? 1 : -1];
typedef char assert_ack_size    [(sizeof(AppCommandAck) == 10u) ? 1 : -1];
原理：

如果 sizeof(AppHeader) == 16，数组大小为 1，编译通过。

否则 ? : 结果为 -1，数组大小为 -1，编译报错。

意义：防止 #pragma pack(1) 被误删或结构体字段被误改，导致主从双方内存布局不一致。

十一、协议层在主从双方的使用
主机侧使用
场景	调用的函数
构造 POLL 帧	app_build_frame(out, APP_MSG_POLL, ...)
构造 COMMAND 帧	app_build_frame(out, APP_MSG_COMMAND, ...)
解析从机响应	app_validate_frame(rx, 32, &h, &p)
处理遥测	读取 AppTelemetry
处理 ACK	读取 AppCommandAck
从机侧使用
场景	调用的函数
解析主机帧	app_validate_frame(rx, 32, &h, &p)
构造遥测帧	app_build_frame(out, APP_MSG_TELEMETRY, ...)
构造 ACK 帧	app_build_frame(out, APP_MSG_COMMAND_ACK, ...)
解析继电器命令	app_relay_apply(current, value, &next)
十二、字节序与内存布局
1. 小端序
STM32 是小端序（Little Endian），memcpy 传输结构体时，多字节字段的低字节在前。例如 0x1234：

内存里是 34 12（低字节在前）。

2. #pragma pack(1)
必须使用，否则编译器可能插入填充字节。例如：

c
/* 没有 pack(1) 时 */
typedef struct {
    uint8_t  a;    /* 1B */
    /* 1B padding */
    uint16_t b;    /* 2B */
} Foo;  /* sizeof = 4 */
c
/* 有 pack(1) 时 */
#pragma pack(push, 1)
typedef struct {
    uint8_t  a;    /* 1B */
    uint16_t b;    /* 2B */
} Foo;  /* sizeof = 3 */
#pragma pack(pop)
主从双方必须都用 pack(1)，否则解析会错位。

3. CRC 的小端序追加
c
out[offset]     = (uint8_t)crc;         /* 低字节 */
out[offset + 1] = (uint8_t)(crc >> 8);  /* 高字节 */
接收端按同样顺序拼回：

c
got_crc = frame[expected-2] | (frame[expected-1] << 8);
十三、错误处理与诊断
1. 组帧失败
app_build_frame 返回 0 表示：

载荷超过 14 字节

总长超过 32 字节

输出指针为空

2. 校验失败
app_validate_frame 返回 0 表示：

帧长不是 32

魔数不匹配

版本不匹配

网络 ID 不匹配

载荷长度超限

CRC 不匹配

3. 上位诊断
主机侧的 gateway_radio_task 会打印：

RF RX DROP format —— 帧格式错误

RF RX DROP id —— 源/目标地址不匹配

RF RX DROP type —— 消息类型不符合预期

从机侧的 radio_task 会打印：

RX DROP: CRC/FORMAT —— 校验失败

RX DROP: ID —— 身份不匹配

十四、完整通信示例
主机下发"继电器 3 吸合"命令
1. 主机组帧：

c
AppCommand c;
c.command_id = 7002;
c.command    = APP_CMD_SET_RELAY;   /* 0x10 */
c.reserved   = 0;
c.value      = 0x0404;              /* channel=3, on=1 */

app_build_frame(tx_frame, APP_MSG_COMMAND, APP_MASTER_ID, 1,
                0xF407, seq, &c, sizeof(c));
生成的 32 字节：

text
[4E 52] [01] [10] [26 20] [00 00] [01 00] [07 F4] [seq] [0A] [00]
[02 1B 00 00] [10] [00] [04 04 00 00]    ← AppCommand
[CRC_LO] [CRC_HI]
[00 00 ... 补齐到 32]
2. 从机解析：

c
app_validate_frame(rx, 32, &h, &p);   /* 校验通过 */
memcpy(&c, p, sizeof(c));
/* c.command == 0x10 */
/* c.value == 0x0404 */
3. 从机执行：

c
app_relay_apply(relay_state, c.value, &next);
/* relay_state=0b0000, next=0b0100 */
relay_write_state(next);
4. 从机回 ACK：

c
AppCommandAck a;
a.command_id = c.command_id;
a.command    = c.command;
a.result     = APP_RESULT_OK;
a.value      = relay_state;   /* 0b0100 */
app_build_frame(out, APP_MSG_COMMAND_ACK, ...);
5. 主机处理 ACK：

校验帧格式；

检查 command_id 是否匹配；

发布命令结果到 MQTT；

如果是继电器命令且成功，发布继电器状态（retained）。

十五、协议的扩展性
1. 增加新命令
只需在 app_protocol.h 里添加新的命令码：

c
APP_CMD_NEW_FEATURE = 0x12
然后在从机 build_response 的 switch 里增加分支。不需要修改帧格式。

2. 增加新载荷类型
只要长度 ≤ 14 字节，可以定义新的结构体：

c
typedef struct {
    uint8_t  xxx;
    uint16_t yyy;
} AppNewPayload;
并在 type 里分配新的消息类型。

3. 升级到多路继电器
当前 app_relay_apply 限制单路。如果要支持多路，只需删除 mask & (mask - 1) == 0 检查，其他逻辑不变。

4. 增加节点数量
node_id 是 16 位，理论上可支持 65535 个节点。实际受限于：

主机的轮询周期（节点越多，轮询间隔越长）；

无线信道容量。