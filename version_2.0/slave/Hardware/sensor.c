#include "sensor.h"
#include "config.h"
#include "app_protocol.h"
#include "i2c_bus.h"
#include <string.h>

/* ------------------------- 内部数据结构定义 ------------------------- */

/* BMP280 校准参数结构体（BMP280 芯片出厂时存储的补偿系数） */
typedef struct {
    uint16_t T1;
    int16_t T2, T3;
    uint16_t P1;
    int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} BmpCalibration;

/* 传感器任务状态 */
enum { SENSOR_IDLE, SENSOR_WAIT_AHT };

/* 全局变量定义 */
static SensorSnapshot snapshot;      /* 最近一次完整传感器快照 */
static BmpCalibration cal;           /* BMP280 校准系数 */
static uint8_t bmp_address;          /* BMP280 实际地址（0x76或0x77，0表示未找到） */
static uint8_t aht_ok;               /* AHT20 最近一次通信成功标志 */
static uint8_t state;                /* 传感器状态机状态 */
static uint32_t deadline_ms;         /* 当前状态机截止时间戳 */
static uint32_t period_ms = APP_DEFAULT_SAMPLE_MS; /* 采样周期（默认值） */
static uint32_t next_sample_ms;      /* 下一次采样时间戳 */
static uint8_t force_pending;        /* 强制采样请求标志 */
static int32_t bmp_t_fine;           /* BMP280 中间温度变量（用于压力补偿） */

/* ------------------------- 小端字节读取辅助函数 ------------------------- */

/* 将两个小端字节组合为无符号16位数，用于BMP280校准参数。 */
static uint16_t u16le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* 将两个小端字节组合为有符号16位数，保留二进制补码符号。 */
static int16_t s16le(const uint8_t *p)
{
    return (int16_t)u16le(p);
}

/* ------------------------- AHT20 CRC8 计算 ------------------------- */

/* 功能：计算AHT20数据手册规定的CRC-8，初值0xFF、多项式0x31。 */
static uint8_t aht_crc8(const uint8_t *p, uint8_t n)
{
    uint8_t crc = 0xFFu, i;

    while (n--) {
        crc ^= *p++;
        for (i = 0; i < 8u; ++i) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ------------------------- BMP280 初始化与采样 ------------------------- */

/*
 * 功能：探测指定BMP280地址，验证芯片ID并读取该芯片独有的校准参数。
 * 返回：1表示初始化成功；0表示无应答、ID错误或校准数据无效。
 */
static uint8_t bmp_init_at(uint8_t address)
{
    uint8_t id, b[24];

    /* 读取芯片ID（寄存器0xD0），应为0x58 */
    if (!i2c_read_regs(address, 0xD0u, &id, 1u) || id != 0x58u) return 0;

    /* 读取24字节校准参数（从0x88开始） */
    if (!i2c_read_regs(address, 0x88u, b, sizeof(b))) return 0;

    /* 解析校准参数（小端） */
    cal.T1 = u16le(b);
    cal.T2 = s16le(b + 2);
    cal.T3 = s16le(b + 4);
    cal.P1 = u16le(b + 6);
    cal.P2 = s16le(b + 8);
    cal.P3 = s16le(b + 10);
    cal.P4 = s16le(b + 12);
    cal.P5 = s16le(b + 14);
    cal.P6 = s16le(b + 16);
    cal.P7 = s16le(b + 18);
    cal.P8 = s16le(b + 20);
    cal.P9 = s16le(b + 22);

    if (!cal.P1) return 0; /* P1 不应为零，否则校准数据无效 */

    /* 配置：sleep 模式下写配置，IIR 滤波系数 x4（寄存器0xF5） */
    if (!i2c_write_reg(address, 0xF5u, 0x08u)) return 0;

    return 1;
}

/*
 * 功能：读取BMP280本轮原始温压数据并执行数据手册64位整数补偿。
 * 输出：temperature单位0.01℃（主要用于内部补偿诊断），pressure单位Pa。
 * 返回：1成功，0表示设备未初始化、I2C失败或除数无效。
 */
static uint8_t bmp_sample(int16_t *temperature, uint32_t *pressure)
{
    uint8_t b[6];
    int32_t adc_t, adc_p, v1, v2, t;
    int64_t p1, p2, p;

    if (!bmp_address) return 0; /* 尚未找到 BMP280 */

    /* 读取6字节原始数据（寄存器0xF7起） */
    if (!i2c_read_regs(bmp_address, 0xF7u, b, 6u)) return 0;

    /* 解析 ADC 值（20位，需拼接） */
    adc_p = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    adc_t = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);

    /* 温度补偿计算（BMP280 数据手册算法） */
    v1 = ((((adc_t >> 3) - ((int32_t)cal.T1 << 1)) * cal.T2) >> 11);
    v2 = ((((((adc_t >> 4) - (int32_t)cal.T1) * ((adc_t >> 4) - (int32_t)cal.T1)) >> 12) * cal.T3) >> 14);
    bmp_t_fine = v1 + v2;
    t = (bmp_t_fine * 5 + 128) >> 8;

    /* 压力补偿计算（BMP280 数据手册算法） */
    p1 = (int64_t)bmp_t_fine - 128000;
    p2 = p1 * p1 * cal.P6;
    p2 = p2 + ((p1 * cal.P5) << 17);
    p2 = p2 + (((int64_t)cal.P4) << 35);
    p1 = ((p1 * p1 * cal.P3) >> 8) + ((p1 * cal.P2) << 12);
    p1 = ((((int64_t)1 << 47) + p1) * cal.P1) >> 33;
    if (!p1) return 0; /* 除数为零 */

    p = 1048576 - adc_p;
    p = (((p << 31) - p2) * 3125) / p1;
    p1 = ((int64_t)cal.P9 * (p >> 13) * (p >> 13)) >> 25;
    p2 = ((int64_t)cal.P8 * p) >> 19;
    p = ((p + p1 + p2) >> 8) + ((int64_t)cal.P7 << 4);

    *temperature = (int16_t)t;
    *pressure = (uint32_t)(p >> 8);
    return 1;
}

/* ------------------------- 传感器公共接口 ------------------------- */

/* 功能：初始化I2C、恢复总线、探测AHT20，并依次探测BMP280的0x76/0x77。 */
void sensors_init(void)
{
    uint8_t status;

    memset(&snapshot, 0, sizeof(snapshot));
    i2c_bus_init();
    (void)i2c_bus_recover();

    /* 探测 AHT20（地址0x38），只读一个字节确认存在 */
    aht_ok = i2c_read(AHT20_I2C_ADDRESS, &status, 1u);

    /* 探测 BMP280，先试主地址0x76，再试备用地址0x77 */
    if (bmp_init_at(BMP280_I2C_ADDRESS_PRIMARY)) {
        bmp_address = BMP280_I2C_ADDRESS_PRIMARY;
    } else if (bmp_init_at(BMP280_I2C_ADDRESS_ALT)) {
        bmp_address = BMP280_I2C_ADDRESS_ALT;
    }

    next_sample_ms = 0u; /* 立即开始首次采样 */
}

/*
 * 功能：非阻塞传感器状态机；由主循环频繁调用。
 * 流程：触发AHT20/BMP280 -> 等待85ms -> 读取和校验 -> 更新完整快照。
 * 注意：等待期间立即返回，因此无线接收不会被AHT20测量时间阻塞。
 */
void sensors_task(uint32_t now)
{
    if (state == SENSOR_IDLE) {
        uint8_t cmd[3] = {0xACu, 0x33u, 0x00u}; /* AHT20 触发测量命令 */

        /* 若没有强制采样且未到下一次采样时间，则直接返回 */
        if (!force_pending && (int32_t)(now - next_sample_ms) < 0) return;

        force_pending = 0u;
        snapshot.status &= SENSOR_MASTER_LOST; /* 清除旧状态，保留主机丢失标志 */

        /* 触发 AHT20 采样 */
        if (i2c_write(AHT20_I2C_ADDRESS, cmd, 3u)) {
            aht_ok = 1u;
            state = SENSOR_WAIT_AHT;
            deadline_ms = now + 85u; /* AHT20 转换需要约80ms */
        } else {
            aht_ok = 0u;
            snapshot.status |= SENSOR_AHT20_ERROR;
            deadline_ms = now + 20u; /* 错误时短等待，进入读取失败处理 */
            state = SENSOR_WAIT_AHT;
        }

        /* 触发 BMP280 采样（若已找到） */
        if (bmp_address) {
            /* 写入 0xF4 寄存器，值0x4D 表示 forced 模式，温度+压力单次测量 */
            if (!i2c_write_reg(bmp_address, 0xF4u, 0x4Du)) {
                snapshot.status |= SENSOR_BMP280_ERROR;
            }
        }
        return;
    }

    /* 若还未到截止时间，继续等待 */
    if ((int32_t)(now - deadline_ms) < 0) return;

    if (state == SENSOR_WAIT_AHT) {
        uint8_t b[7];
        int16_t bmp_temp;
        uint32_t raw_h, raw_t, pressure;

        /* 读取 AHT20 数据和状态 */
        if (aht_ok && i2c_read(AHT20_I2C_ADDRESS, b, 7u) && !(b[0] & 0x80u)) {
            raw_h = ((uint32_t)b[1] << 12) | ((uint32_t)b[2] << 4) | (b[3] >> 4);
            raw_t = ((uint32_t)(b[3] & 0x0Fu) << 16) | ((uint32_t)b[4] << 8) | b[5];
            snapshot.humidity_centi_rh = (uint16_t)(((uint64_t)raw_h * 10000u + 524288u) / 1048576u);
            snapshot.temperature_centi_c = (int16_t)(((int64_t)raw_t * 20000 + 524288) / 1048576 - 5000);
            snapshot.status |= SENSOR_AHT20_OK;

            /* CRC 校验（AHT20 数据手册 CRC-8） */
            if (aht_crc8(b, 6u) == b[6]) {
                snapshot.status |= SENSOR_AHT20_CRC_OK;
            } else {
                snapshot.status |= SENSOR_AHT20_ERROR;
            }
        } else {
            snapshot.status |= SENSOR_AHT20_ERROR;
        }

        /* 读取 BMP280 数据 */
        if (bmp_sample(&bmp_temp, &pressure)) {
            snapshot.pressure_pa = pressure;
            snapshot.status |= SENSOR_BMP280_OK;
        } else {
            snapshot.status |= SENSOR_BMP280_ERROR;
        }

        snapshot.status |= SENSOR_DATA_FRESH;
        snapshot.sampled_at_ms = now;
        next_sample_ms = now + period_ms;
        state = SENSOR_IDLE;
    }
}

/* 请求尽快开始一次新采样；若当前正在测量，则下一轮再执行。 */
void sensors_force_sample(void)
{
    force_pending = 1u;
}

/* 更新周期采样间隔；范围校验由命令业务层负责。 */
void sensors_set_period(uint32_t v)
{
    period_ms = v;
}

/* 返回当前采样周期，供诊断信息和命令回执使用。 */
uint32_t sensors_get_period(void)
{
    return period_ms;
}

/* 复制最近一次完整快照，避免无线层直接依赖传感器内部状态。 */
void sensors_get_snapshot(SensorSnapshot *out)
{
    *out = snapshot;
}

/* 返回AHT20最近一次探测/通信是否成功。 */
uint8_t sensors_aht_address_ok(void)
{
    return aht_ok;
}

/* 返回探测到的BMP280地址；返回0表示未找到。 */
uint8_t sensors_bmp_address(void)
{
    return bmp_address;
}