#include "app_protocol.h"
#include <string.h>

/*
 * 功能：计算应用层CRC-16/CCITT-FALSE，用于发现无线硬件CRC之外的数据错误。
 * 参数：data为待校验数据，length为字节数。
 * 返回：16位CRC值，初始值0xFFFF，多项式0x1021。
 */
uint16_t app_crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFu;
    uint8_t i;
    while (length--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (i = 0; i < 8; ++i) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/*
 * 功能：构造固定32字节应用帧，写入帧头、业务载荷及CRC，剩余空间补零。
 * 参数：out至少32字节；type/src/dst/boot/seq描述消息；payload可为NULL。
 * 返回：成功返回32，载荷超限或帧超限返回0。
 */
uint8_t app_build_frame(uint8_t *out, uint8_t type, uint16_t src_id,   uint16_t dst_id, uint16_t boot_id, uint16_t seq, const void *payload, uint8_t payload_len)
{
    AppHeader h;
    uint16_t crc;
    uint8_t total;

    if (payload_len > APP_MAX_PAYLOAD) return 0;
    total = (uint8_t)(sizeof(AppHeader) + payload_len + 2u);
    if (total > APP_RADIO_MAX_FRAME) return 0;

    memset(out, 0, APP_RADIO_MAX_FRAME);
    h.magic = APP_MAGIC;
    h.version = APP_VERSION;
    h.type = type;
    h.network_id = APP_NETWORK_ID;
    h.src_id = src_id;
    h.dst_id = dst_id;
    h.boot_id = boot_id;
    h.seq = seq;
    h.payload_len = payload_len;
    h.flags = 0;
    memcpy(out, &h, sizeof(h));
    if (payload_len && payload) 
    memcpy(out + sizeof(h), payload, payload_len);
    crc = app_crc16(out, (uint8_t)(sizeof(h) + payload_len));
    out[sizeof(h) + payload_len] = (uint8_t)crc;
    out[sizeof(h) + payload_len + 1u] = (uint8_t)(crc >> 8);
    return APP_RADIO_MAX_FRAME;
}

/*
 * 功能：检查固定帧长度、魔数、版本、网络号、载荷长度和应用CRC。
 * 参数：header和payload用于返回帧内只读指针，生命周期与frame相同。
 * 返回：1表示合法，0表示应丢弃；身份src/dst由上层业务继续检查。
 */
uint8_t app_validate_frame(const uint8_t *frame, uint8_t frame_len, const AppHeader **header, const uint8_t **payload)
{
    const AppHeader *h;
    uint8_t expected;
    uint16_t got_crc, calculated;

    if (frame_len != APP_RADIO_MAX_FRAME) return 0;
    h = (const AppHeader *)frame;
    expected = (uint8_t)(sizeof(AppHeader) + h->payload_len + 2u);
    if (h->magic != APP_MAGIC || h->version != APP_VERSION ||
        h->network_id != APP_NETWORK_ID || h->payload_len > APP_MAX_PAYLOAD ||
        expected > APP_RADIO_MAX_FRAME) return 0;
    got_crc = (uint16_t)frame[expected - 2u] | ((uint16_t)frame[expected - 1u] << 8);
    calculated = app_crc16(frame, (uint8_t)(expected - 2u));
    if (got_crc != calculated) return 0;
    *header = h;
    *payload = frame + sizeof(AppHeader);
    return 1;
}
