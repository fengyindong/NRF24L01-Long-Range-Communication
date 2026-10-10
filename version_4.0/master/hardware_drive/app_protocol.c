#include "app_protocol.h"
#include <string.h>

/* ============================================================
 * CRC-16 校验
 * ============================================================ */

/**
 * @brief 计算 CRC-16/CCITT-FALSE 校验值。
 *
 * 算法参数：初始值 0xFFFF，多项式 0x1021，
 * 输入数据不反转，结果不异或。
 *
 * @param data   待校验的数据缓冲区
 * @param length 数据长度（字节）
 * @return 16 位 CRC 校验值
 */
uint16_t app_crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFu;
    uint8_t  i;

    while (length--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (i = 0u; i < 8u; ++i) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ============================================================
 * 帧构建
 * ============================================================ */

/**
 * @brief 构造一个固定 32 字节的应用帧。
 *
 * 帧格式：帧头(16B) + 载荷(0~14B) + CRC(2B) + 填充(全 0)，
 * 总长度固定为 APP_RADIO_MAX_FRAME (32 字节)。
 *
 * @param out         输出缓冲区，至少 32 字节
 * @param type        消息类型（APP_MSG_xxx）
 * @param src_id      源节点 ID
 * @param dst_id      目标节点 ID
 * @param boot_id     节点启动序号
 * @param seq         帧序列号
 * @param payload     载荷数据指针，可为 NULL（当 payload_len 为 0 时）
 * @param payload_len 载荷长度，最大 APP_MAX_PAYLOAD (14)
 * @return 成功返回 32，参数非法或长度超限返回 0
 */
uint8_t app_build_frame(uint8_t *out, uint8_t type, uint16_t src_id,
                        uint16_t dst_id, uint16_t boot_id, uint16_t seq,
                        const void *payload, uint8_t payload_len)
{
    AppHeader header;
    uint16_t  crc;
    uint8_t   total;

    /* 参数合法性检查：输出指针非空、载荷不超限、有载荷时指针非空 */
    if (!out || payload_len > APP_MAX_PAYLOAD || (payload_len && !payload)) {
        return 0u;
    }

    /* 计算总帧长：帧头 + 载荷 + 2 字节 CRC */
    total = (uint8_t)(sizeof(header) + payload_len + 2u);
    if (total > APP_RADIO_MAX_FRAME) {
        return 0u;
    }

    /* 清空输出缓冲区，确保填充部分为 0 */
    memset(out, 0, APP_RADIO_MAX_FRAME);

    /* 填充帧头 */
    header.magic       = APP_MAGIC;
    header.version     = APP_VERSION;
    header.type        = type;
    header.network_id  = APP_NETWORK_ID;
    header.src_id      = src_id;
    header.dst_id      = dst_id;
    header.boot_id     = boot_id;
    header.seq         = seq;
    header.payload_len = payload_len;
    header.flags       = 0u;

    memcpy(out, &header, sizeof(header));

    /* 拷贝载荷（如果有） */
    if (payload_len) {
        memcpy(out + sizeof(header), payload, payload_len);
    }

    /* 计算并追加 CRC：低字节在前，高字节在后（小端序） */
    crc = app_crc16(out, (uint8_t)(sizeof(header) + payload_len));
    out[sizeof(header) + payload_len]      = (uint8_t)crc;
    out[sizeof(header) + payload_len + 1u] = (uint8_t)(crc >> 8);

    return APP_RADIO_MAX_FRAME;
}

/* ============================================================
 * 帧校验
 * ============================================================ */

/**
 * @brief 验证接收到的帧格式和 CRC。
 *
 * 检查项目：
 *   - 帧长必须为 APP_RADIO_MAX_FRAME (32)
 *   - 魔数、版本、网络 ID 必须匹配
 *   - 载荷长度不能超限
 *   - CRC 必须一致
 *
 * @param frame     待验证的帧数据
 * @param frame_len 帧长度（必须为 32）
 * @param header    输出参数，返回帧头指针
 * @param payload   输出参数，返回载荷指针
 * @return 验证通过返回 1，失败返回 0
 */
uint8_t app_validate_frame(const uint8_t *frame, uint8_t frame_len,
                           const AppHeader **header, const uint8_t **payload)
{
    const AppHeader *parsed;
    uint8_t          expected;
    uint16_t         received_crc;

    /* 空指针与长度检查 */
    if (!frame || !header || !payload || frame_len != APP_RADIO_MAX_FRAME) {
        return 0u;
    }

    parsed   = (const AppHeader *)frame;
    expected = (uint8_t)(sizeof(AppHeader) + parsed->payload_len + 2u);

    /* 帧头字段合法性检查 */
    if (parsed->magic != APP_MAGIC ||
        parsed->version != APP_VERSION ||
        parsed->network_id != APP_NETWORK_ID ||
        parsed->payload_len > APP_MAX_PAYLOAD ||
        expected > APP_RADIO_MAX_FRAME) {
        return 0u;
    }

    /* 提取接收到的 CRC 并重新计算对比 */
    received_crc = (uint16_t)frame[expected - 2u] |
                   ((uint16_t)frame[expected - 1u] << 8);

    if (received_crc != app_crc16(frame, (uint8_t)(expected - 2u))) {
        return 0u;
    }

    /* 输出解析结果（生命周期与 frame 相同） */
    *header  = parsed;
    *payload = frame + sizeof(AppHeader);
    return 1u;
}

/* ============================================================
 * 继电器命令编码 / 解码
 * ============================================================ */

/**
 * @brief 编码单路继电器命令。
 *
 * 生成命令值：低 4 位为通道掩码（仅一位为 1），
 * 高 8 位中对应位表示目标状态（1=吸合，0=断开）。
 *
 * @param channel 通道号（1~4）
 * @param on      目标状态（0=断开，1=吸合）
 * @param value   输出参数，返回编码后的命令值
 * @return 成功返回 1，参数无效返回 0
 */
uint8_t app_relay_encode(uint8_t channel, uint8_t on, uint32_t *value)
{
    uint8_t bit;

    if (!value || channel < 1u || channel > 4u || on > 1u) {
        return 0u;
    }

    bit    = (uint8_t)(1u << (channel - 1u));
    *value = (uint32_t)bit | (on ? ((uint32_t)bit << 8) : 0u);
    return 1u;
}

/**
 * @brief 解析并应用继电器命令值，计算新的继电器状态。
 *
 * 命令值格式：低 4 位为通道掩码，高 8 位中对应位为目标状态。
 * 新状态 = (当前状态 & ~mask) | (目标状态 & mask)。
 *
 * 注：本版本要求 mask 只能为单个通道（必须为 2 的幂），
 * 即一次命令只操作一路继电器。
 *
 * @param current_state 当前继电器状态（低 4 位有效）
 * @param value         命令值
 * @param next_state    输出参数，返回计算后的新状态
 * @return 成功返回 1，参数无效返回 0
 */
uint8_t app_relay_apply(uint8_t current_state, uint32_t value,
                        uint8_t *next_state)
{
    uint8_t mask;
    uint8_t requested;

    /* 检查输出指针，并确保 value 只使用了低 4 位和 bit8~11 */
    if (!next_state || (value & ~0x0F0Fu)) {
        return 0u;
    }

    mask      = (uint8_t)(value & 0x0Fu);
    requested = (uint8_t)((value >> 8) & 0x0Fu);

    /* 掩码不能为 0，且必须是单个通道（2 的幂）；
     * 目标状态不能超出掩码范围 */
    if (!mask || (mask & (uint8_t)(mask - 1u)) ||
        (requested & (uint8_t)~mask)) {
        return 0u;
    }

    *next_state = (uint8_t)(((current_state & (uint8_t)~mask) |
                             (requested & mask)) & 0x0Fu);
    return 1u;
}