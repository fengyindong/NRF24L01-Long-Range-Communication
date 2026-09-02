#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

typedef struct {
    int16_t temperature_centi_c;
    uint16_t humidity_centi_rh;
    uint32_t pressure_pa;
    uint8_t status;
    uint32_t sampled_at_ms;
} SensorSnapshot;

/* 初始化I2C并探测AHT20、BMP280。 */
void sensors_init(void);
/* 推进非阻塞采集状态机，需在主循环中频繁调用。 */
void sensors_task(uint32_t now_ms);
/* 请求尽快执行一次新采样。 */
void sensors_force_sample(void);
/* 设置采样周期，调用方负责范围校验。 */
void sensors_set_period(uint32_t period_ms);
/* 读取当前采样周期，单位ms。 */
uint32_t sensors_get_period(void);
/* 复制最近一次完整数据快照。 */
void sensors_get_snapshot(SensorSnapshot *out);
/* 返回AHT20最近通信是否正常。 */
uint8_t sensors_aht_address_ok(void);
/* 返回BMP280实际地址0x76/0x77，未探测到返回0。 */
uint8_t sensors_bmp_address(void);

#endif
